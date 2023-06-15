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


/** Note: Only left-side registers are listed. Right-side IDs are equal, send to both! */
uint8_t _INV_READ_REG_QUEUE[] = {
    INVERTERS_INV_L_SEND_READ_ID_40H_STATUS_MAP_CHOICE,
    INVERTERS_INV_L_SEND_READ_ID_49H_T_MOTOR_CHOICE,
    INVERTERS_INV_L_SEND_READ_ID_4AH_T_IGBT_CHOICE,
    INVERTERS_INV_L_SEND_READ_ID_51H_KERN_MODE_STATE_CHOICE,
    INVERTERS_INV_L_SEND_READ_ID_8FH_ERRORWARNING_MAP_CHOICE,
    INVERTERS_INV_L_SEND_READ_ID_A8H_N_ACTUAL_FILT_CHOICE,
    INVERTERS_INV_L_SEND_READ_ID_D8H_LOGICREADBITSIN_OUT_CHOICE,
};
uint8_t _INV_READ_REG_QUEUE_LEN = sizeof(_INV_READ_REG_QUEUE) / sizeof(_INV_READ_REG_QUEUE[0]);
uint8_t _INV_read_reg_queue_idx = 0;


void INV_parse_CAN_msg(uint8_t *buf, uint8_t len) {
    LOG_write(LOGLEVEL_DEBUG, "[INV] Received: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
        buf[0], buf[1], buf[2], buf[3], buf[4]
    );

    if (buf[0] == INVERTERS_INV_L_RCV_FRAME_ID) {
        inverters_inv_l_rcv_t tmp;
        inverters_inv_l_rcv_unpack(&tmp, buf, len);
        inverters_inv_l_rcv_raw_to_conversion_struct(&_INV_l_recv, &tmp);
    } else if (buf[0] == INVERTERS_INV_R_RCV_FRAME_ID) {
        inverters_inv_r_rcv_t tmp;
        inverters_inv_r_rcv_unpack(&tmp, buf, len);
        inverters_inv_r_rcv_raw_to_conversion_struct(&_INV_r_recv, &tmp);
    }
}


void INV_read_next_register() {
    CAN_MessageTypeDef msg = { 0U };
    uint8_t reg = _INV_READ_REG_QUEUE[_INV_read_reg_queue_idx];
    _INV_read_reg_queue_idx = (_INV_read_reg_queue_idx + 1) % _INV_READ_REG_QUEUE_LEN;

    /* LEFT side */
    _INV_l_send.send_mux = INVERTERS_INV_L_SEND_SEND_MUX_ID_3D_READ_CMD_CHOICE;
    _INV_l_send.read_id = reg;
    inverters_inv_l_send_t tmp_l;
    inverters_inv_l_send_conversion_to_raw_struct(&tmp_l, &_INV_l_send);
    msg.size = inverters_inv_l_send_pack(msg.data, &tmp_l, 8);
    msg.id = INVERTERS_INV_L_SEND_FRAME_ID;
    
    LOG_write(LOGLEVEL_DEBUG, "[INV] Sending: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
        msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]
    );

    CAN_send(&msg, &hcan1);

    /* RIGHT side */
    _INV_r_send.send_mux = INVERTERS_INV_R_SEND_SEND_MUX_ID_3D_READ_CMD_CHOICE;
    _INV_r_send.read_id = reg;
    inverters_inv_r_send_t tmp_r;
    inverters_inv_r_send_conversion_to_raw_struct(&tmp_r, &_INV_r_send);
    msg.size = inverters_inv_r_send_pack(msg.data, &tmp_r, 8);
    msg.id = INVERTERS_INV_R_SEND_FRAME_ID;

    LOG_write(LOGLEVEL_DEBUG, "[INV] Sending: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
        msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]
    );

    CAN_send(&msg, &hcan1);
}

float INV_get_inv_temp(INV_SideTypeDef side) {
    uint16_t raw_temp = (side == INV_LEFT) ? _INV_l_recv.t_igbt : _INV_r_recv.t_igbt;
    return 0.005f * raw_temp - 38.0f;
}
