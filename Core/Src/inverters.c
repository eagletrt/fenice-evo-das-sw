#include "can.h"
#include "can_user_functions.h"
#include "can_messages.h"
#include "inverters.h"
#include "logger.h"
#include "../Lib/can/lib/inverters/inverters_network.h"


inverters_inv_r_send_converted_t _INV_r_send;
inverters_inv_l_send_converted_t _INV_l_send;
inverters_inv_r_rcv_converted_t _INV_r_recv;
inverters_inv_l_rcv_converted_t _INV_l_recv;



/** Note: Only left-side registers are listed. Right-side IDs are equal, but remember to send to both! */
typedef struct {
    uint8_t id;
    INV_SideTypeDef side;
    uint16_t interval_ms;
    uint32_t last_read_ms;
} INV_RegMetadataTypeDef;

INV_RegMetadataTypeDef _INV_READ_REG_QUEUE[] = {
    { INVERTERS_INV_L_SEND_READ_ID_22H_I_CMD_RAMP_CHOICE,          INV_LEFT,  20,  0  },
    { INVERTERS_INV_R_SEND_READ_ID_26H_I_CMD_CHOICE,               INV_RIGHT, 20,  1  },
    { INVERTERS_INV_L_SEND_READ_ID_22H_I_CMD_RAMP_CHOICE,          INV_LEFT,  20,  2  },
    { INVERTERS_INV_R_SEND_READ_ID_26H_I_CMD_CHOICE,               INV_RIGHT, 20,  3  },
    { INVERTERS_INV_L_SEND_READ_ID_27H_IQ_ACTUAL_CHOICE,           INV_LEFT,  20,  4  },
    { INVERTERS_INV_R_SEND_READ_ID_27H_IQ_ACTUAL_CHOICE,           INV_RIGHT, 20,  5  },
    { INVERTERS_INV_L_SEND_READ_ID_40H_STATUS_MAP_CHOICE,          INV_LEFT,  20,  6  },
    { INVERTERS_INV_R_SEND_READ_ID_40H_STATUS_MAP_CHOICE,          INV_RIGHT, 20,  7  },
    { INVERTERS_INV_L_SEND_READ_ID_49H_T_MOTOR_CHOICE,             INV_LEFT,  100, 8  },
    { INVERTERS_INV_R_SEND_READ_ID_49H_T_MOTOR_CHOICE,             INV_RIGHT, 100, 9  },
    { INVERTERS_INV_L_SEND_READ_ID_4AH_T_IGBT_CHOICE,              INV_LEFT,  100, 10 },
    { INVERTERS_INV_R_SEND_READ_ID_4AH_T_IGBT_CHOICE,              INV_RIGHT, 100, 11 },
    { INVERTERS_INV_L_SEND_READ_ID_51H_KERN_MODE_STATE_CHOICE,     INV_LEFT,  100, 12 },
    { INVERTERS_INV_R_SEND_READ_ID_51H_KERN_MODE_STATE_CHOICE,     INV_RIGHT, 100, 13 },
    { INVERTERS_INV_L_SEND_READ_ID_8FH_ERRORWARNING_MAP_CHOICE,    INV_LEFT,  100, 14 },
    { INVERTERS_INV_R_SEND_READ_ID_8FH_ERRORWARNING_MAP_CHOICE,    INV_RIGHT, 100, 15 },
    { INVERTERS_INV_L_SEND_READ_ID_A8H_N_ACTUAL_FILT_CHOICE,       INV_LEFT,  20,  16 },
    { INVERTERS_INV_R_SEND_READ_ID_A8H_N_ACTUAL_FILT_CHOICE,       INV_RIGHT, 20,  17 },
    { INVERTERS_INV_L_SEND_READ_ID_D8H_LOGICREADBITSIN_OUT_CHOICE, INV_LEFT,  20,  18 },
    { INVERTERS_INV_R_SEND_READ_ID_D8H_LOGICREADBITSIN_OUT_CHOICE, INV_RIGHT, 20,  19 },
};
uint8_t _INV_READ_REG_QUEUE_LEN = sizeof(_INV_READ_REG_QUEUE) / sizeof(_INV_READ_REG_QUEUE[0]);
uint8_t _INV_last_read_reg_idx = 0;


void INV_parse_CAN_msg(CAN_IdTypeDef id, uint8_t *buf, uint8_t len) {
    // LOG_write(LOGLEVEL_DEBUG, "[INV] Received: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
    //     id, buf[0], buf[1], buf[2], buf[3], buf[4]
    // );

    if (id == INVERTERS_INV_L_RCV_FRAME_ID) {
        inverters_inv_l_rcv_t tmp;
        inverters_inv_l_rcv_unpack(&tmp, buf, len);
        inverters_inv_l_rcv_raw_to_conversion_struct(&_INV_l_recv, &tmp);
    } else if (id == INVERTERS_INV_R_RCV_FRAME_ID) {
        inverters_inv_r_rcv_t tmp;
        inverters_inv_r_rcv_unpack(&tmp, buf, len);
        inverters_inv_r_rcv_raw_to_conversion_struct(&_INV_r_recv, &tmp);
    }
}

void INV_read_next_register() {
    INV_RegMetadataTypeDef *reg_to_update;
    bool found = false;

    for (uint8_t i = 0; i < _INV_READ_REG_QUEUE_LEN && !found; i++) {
        uint8_t tmp_idx = (_INV_last_read_reg_idx + i + 1) % _INV_READ_REG_QUEUE_LEN;
        INV_RegMetadataTypeDef *tmp_reg = &(_INV_READ_REG_QUEUE[tmp_idx]);

        if (HAL_GetTick() - tmp_reg->last_read_ms > tmp_reg->interval_ms) {
            reg_to_update = tmp_reg;
            reg_to_update->last_read_ms = HAL_GetTick();
            _INV_last_read_reg_idx = tmp_idx;
            found = true;
        }
    }

    if (!found) {
        return; // No updates needed
    }

    CAN_MessageTypeDef msg = { 0U };

    if (reg_to_update->side == INV_LEFT) {
        _INV_l_send.send_mux = INVERTERS_INV_L_SEND_SEND_MUX_ID_3D_READ_CMD_CHOICE;
        _INV_l_send.read_id = reg_to_update->id;

        inverters_inv_l_send_t tmp_l;
        inverters_inv_l_send_conversion_to_raw_struct(&tmp_l, &_INV_l_send);
        msg.size = inverters_inv_l_send_pack(msg.data, &tmp_l, 8);
        msg.id = INVERTERS_INV_L_SEND_FRAME_ID;
    } else {
        _INV_r_send.send_mux = INVERTERS_INV_R_SEND_SEND_MUX_ID_3D_READ_CMD_CHOICE;
        _INV_r_send.read_id = reg_to_update->id;

        inverters_inv_r_send_t tmp_r;
        inverters_inv_r_send_conversion_to_raw_struct(&tmp_r, &_INV_r_send);
        msg.size = inverters_inv_r_send_pack(msg.data, &tmp_r, 8);
        msg.id = INVERTERS_INV_R_SEND_FRAME_ID;
    }

    CAN_send(&msg, &hcan1);
}

float INV_get_IGBT_temp(INV_SideTypeDef side) {
    uint16_t raw_temp = (side == INV_LEFT) ? _INV_l_recv.t_igbt : _INV_r_recv.t_igbt;
    return 0.005f * raw_temp - 38.0f;
}

float INV_get_motor_temp(INV_SideTypeDef side) {
    uint16_t raw_temp = (side == INV_LEFT) ? _INV_l_recv.t_motor : _INV_r_recv.t_motor;
    return (raw_temp - 9393.9f) / 55.1f;
}

int16_t INV_get_RPM(INV_SideTypeDef side) {
    float raw_rpm = (side == INV_LEFT) ? _INV_l_recv.n_actual : _INV_r_recv.n_actual;
    return (float)raw_rpm; // TODO: Probably needs conversion!
}

bool INV_is_drive_enabled(INV_SideTypeDef side) {
    return (side == INV_LEFT) ? _INV_l_recv.ena82 : _INV_r_recv.ena82;
}

bool INV_get_RFE_state(INV_SideTypeDef side) {
    return (side == INV_LEFT) ? _INV_l_recv.rfe216 : _INV_r_recv.rfe216;
}

bool INV_get_FRG_state(INV_SideTypeDef side) {
    return (side == INV_LEFT) ? _INV_l_recv.frgrun : _INV_r_recv.frgrun;
}