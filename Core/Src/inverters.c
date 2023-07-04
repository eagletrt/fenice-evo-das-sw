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
    uint32_t interval_ms;
    uint32_t last_read_ms;
} INV_RegMetadataTypeDef;

INV_RegMetadataTypeDef _INV_READ_REG_QUEUE[] = {
    { INVERTERS_INV_L_SEND_READ_ID_22H_I_CMD_RAMP_CHOICE,          INV_LEFT,  50,  0  * 2},
    { INVERTERS_INV_R_SEND_READ_ID_22H_I_CMD_RAMP_CHOICE,          INV_RIGHT, 50,  1  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_26H_I_CMD_CHOICE,               INV_LEFT,  50,  2  * 2}, 
    { INVERTERS_INV_R_SEND_READ_ID_26H_I_CMD_CHOICE,               INV_RIGHT, 50,  3  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_27H_IQ_ACTUAL_CHOICE,           INV_LEFT,  50,  4  * 2},
    { INVERTERS_INV_R_SEND_READ_ID_27H_IQ_ACTUAL_CHOICE,           INV_RIGHT, 50,  5  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_40H_STATUS_MAP_CHOICE,          INV_LEFT,  50,  6  * 2},
    { INVERTERS_INV_R_SEND_READ_ID_40H_STATUS_MAP_CHOICE,          INV_RIGHT, 50,  7  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_49H_T_MOTOR_CHOICE,             INV_LEFT,  100, 8  * 2},
    { INVERTERS_INV_R_SEND_READ_ID_49H_T_MOTOR_CHOICE,             INV_RIGHT, 100, 9  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_4AH_T_IGBT_CHOICE,              INV_LEFT,  100, 10 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_4AH_T_IGBT_CHOICE,              INV_RIGHT, 100, 11 * 2},
    { INVERTERS_INV_L_SEND_READ_ID_51H_KERN_MODE_STATE_CHOICE,     INV_LEFT,  100, 12 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_51H_KERN_MODE_STATE_CHOICE,     INV_RIGHT, 100, 13 * 2},

    { INVERTERS_INV_L_SEND_READ_ID_52H_STATUS_MASK_CHOICE,         INV_LEFT,  100, 14 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_52H_STATUS_MASK_CHOICE,         INV_RIGHT, 100, 15 * 2},

    { INVERTERS_INV_L_SEND_READ_ID_8FH_ERRORWARNING_MAP_CHOICE,    INV_LEFT,  100, 16 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_8FH_ERRORWARNING_MAP_CHOICE,    INV_RIGHT, 100, 17 * 2},
    { INVERTERS_INV_L_SEND_READ_ID_A8H_N_ACTUAL_FILT_CHOICE,       INV_LEFT,  50,  18 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_A8H_N_ACTUAL_FILT_CHOICE,       INV_RIGHT, 50,  19 * 2},
    { INVERTERS_INV_L_SEND_READ_ID_D8H_LOGICREADBITSIN_OUT_CHOICE, INV_LEFT,  50,  20 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_D8H_LOGICREADBITSIN_OUT_CHOICE, INV_RIGHT, 50,  21 * 2},
};
uint8_t _INV_READ_REG_QUEUE_LEN = sizeof(_INV_READ_REG_QUEUE) / sizeof(_INV_READ_REG_QUEUE[0]);
uint8_t _INV_last_read_reg_idx = 0;


/**
 * Serialize the `inverters_inv_*_send_converted_t` struct and send it in CAN
*/
void _INV_send_CAN_msg(INV_SideTypeDef side) {
    CAN_MessageTypeDef msg = { 0U };

    if (side == INV_LEFT) {
        inverters_inv_l_send_t tmp_l;
        inverters_inv_l_send_conversion_to_raw_struct(&tmp_l, &_INV_l_send);
        msg.size = inverters_inv_l_send_pack(msg.data, &tmp_l, INVERTERS_INV_L_SEND_BYTE_SIZE);
        msg.id = INVERTERS_INV_L_SEND_FRAME_ID;
    } else {
        inverters_inv_r_send_t tmp_r;
        inverters_inv_r_send_conversion_to_raw_struct(&tmp_r, &_INV_r_send);
        msg.size = inverters_inv_r_send_pack(msg.data, &tmp_r, INVERTERS_INV_R_SEND_BYTE_SIZE);
        msg.id = INVERTERS_INV_R_SEND_FRAME_ID;
    }

    CAN_send(&msg, &hcan1);
}


/**
 * Deserialize the gieven CAN message into the `inverters_inv_*_rcv_converted_t` struct
*/
void INV_parse_CAN_msg(CAN_IdTypeDef id, uint8_t *buf, uint8_t len) {
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

/**
 * Ierate over the registers queue and find the next one to read, starting from where the function
 * left at the last call.
*/
void INV_read_next_register() {
    INV_RegMetadataTypeDef *reg_to_update;
    bool found = false;

    for (int i = 0; i < _INV_READ_REG_QUEUE_LEN; i++) {
        int tmp_idx = (_INV_last_read_reg_idx + i + 1) % _INV_READ_REG_QUEUE_LEN;
        INV_RegMetadataTypeDef *tmp_reg = &(_INV_READ_REG_QUEUE[tmp_idx]);

        if (HAL_GetTick() - tmp_reg->last_read_ms > tmp_reg->interval_ms) {
            reg_to_update = tmp_reg;
            reg_to_update->last_read_ms = HAL_GetTick();
            _INV_last_read_reg_idx = tmp_idx;
            found = true;
            break;
        }
    }

    if (!found) {
        return; // No updates needed
    }

    if (reg_to_update->side == INV_LEFT) {
        _INV_l_send.send_mux = INVERTERS_INV_L_SEND_SEND_MUX_ID_3D_READ_CMD_CHOICE;
        _INV_l_send.read_id = reg_to_update->id;
        _INV_send_CAN_msg(INV_LEFT);
    } else {
        _INV_r_send.send_mux = INVERTERS_INV_R_SEND_SEND_MUX_ID_3D_READ_CMD_CHOICE;
        _INV_r_send.read_id = reg_to_update->id;
        _INV_send_CAN_msg(INV_RIGHT);
    }
}

/**
 * Convert the torque in Nm to the current in A_rms
*/
float INV_torque_to_current(float torque){
    return torque / MOT_TORQUE_COEFF;
}

/**
 * @brief     Ensure that power doesn't exceed 40kW per motor by damping the current request with RPMs
 * @param[in] request The requested current in Nm
 * @param[in] rpm     The motor RPMs
 * @return    The current in Nm after the power limit has been applied
 */
float INV_cutoff_torque(float request, float rpm) {
    if (rpm < INV_RPM_CUTOFF)
        return request;
    
    /* See Mattia Piccini's report for details */
    float lim_current = (float)INV_CUTOFF_COEFF_TORQUE / (float)rpm;
    
    return (request > lim_current && lim_current > 0) ? lim_current : request;
}

int16_t INV_current_to_num(float current) {
    /* Check Inverter datasheet */
    //A maximum value of 32.768 equals to 169.9A max absorbed
    int16_t max_current = INT16_MAX;
    float num = (max_current / INV_CURR_PEAK_REAL) * current;
    return (int16_t)num; 
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
    return (side == INV_LEFT) ? _INV_l_recv.ena64 : _INV_r_recv.ena64;
}

bool INV_get_RFE_state(INV_SideTypeDef side) {
    return (side == INV_LEFT) ? _INV_l_recv.rfe216 : _INV_r_recv.rfe216;
}

bool INV_get_FRG_state(INV_SideTypeDef side) {
    return (side == INV_LEFT) ? _INV_l_recv.frgrun : _INV_r_recv.frgrun;
}

void INV_enable_drive(INV_SideTypeDef side) {
    if (side == INV_LEFT) {
        _INV_l_send.send_mux = INVERTERS_INV_L_SEND_SEND_MUX_ID_51_KERN_MODE_STATE_CHOICE;
        _INV_l_send.km_frg_off = 0;
        _INV_send_CAN_msg(INV_LEFT);
    } else {
        _INV_r_send.send_mux = INVERTERS_INV_R_SEND_SEND_MUX_ID_51_KERN_MODE_STATE_CHOICE;
        _INV_r_send.km_frg_off = 0;
        _INV_send_CAN_msg(INV_RIGHT);
    }
}

void INV_disable_drive(INV_SideTypeDef side) {
    if (side == INV_LEFT) {
        _INV_l_send.send_mux = INVERTERS_INV_L_SEND_SEND_MUX_ID_51_KERN_MODE_STATE_CHOICE;
        _INV_l_send.km_frg_off = 1;
        _INV_send_CAN_msg(INV_LEFT);
    } else {
        _INV_r_send.send_mux = INVERTERS_INV_R_SEND_SEND_MUX_ID_51_KERN_MODE_STATE_CHOICE;
        _INV_r_send.km_frg_off = 1;
        _INV_send_CAN_msg(INV_RIGHT);
    }
}

void INV_set_torque_Nm(INV_SideTypeDef side, float torque) {
    float current = INV_torque_to_current(torque);
    int16_t num = INV_current_to_num(current);
    if (side == INV_LEFT) {
        _INV_l_send.send_mux = INVERTERS_INV_L_SEND_SEND_MUX_ID_90_M_SETDIG_CHOICE;
        _INV_l_send.m_setdig__iq = num;
        _INV_send_CAN_msg(INV_LEFT);
    } else if (side == INV_RIGHT) {
        _INV_r_send.send_mux = INVERTERS_INV_R_SEND_SEND_MUX_ID_90_M_SETDIG_CHOICE;
        _INV_r_send.m_setdig__iq = num;
        _INV_send_CAN_msg(INV_RIGHT);
    }
}
