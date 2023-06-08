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



void INV_parse_CAN_msg(uint8_t *buf, uint8_t len) {

}


void INV_send_autoTX_req(INV_SideTypeDef side) {
    CAN_MessageTypeDef msg = { 0U };

    if (side == INV_LEFT) {
        inverters_inv_l_send_t tmp;
        tmp.send_mux = INVERTERS_INV_L_SEND_READ_ID_40H_STATUS_MAP_CHOICE;
        inverters_inv_l_send_conversion_to_raw_struct(&tmp, &_INV_l_send);
        msg.size = inverters_inv_l_send_pack(msg.data, &tmp, 8);
        msg.id = INVERTERS_INV_L_SEND_FRAME_ID;
    } else {
        inverters_inv_r_send_t tmp;
        tmp.send_mux = INVERTERS_INV_R_SEND_READ_ID_40H_STATUS_MAP_CHOICE;
        inverters_inv_r_send_conversion_to_raw_struct(&tmp, &_INV_r_send);
        msg.size = inverters_inv_r_send_pack(msg.data, &tmp, 8);
        msg.id = INVERTERS_INV_R_SEND_FRAME_ID;
    }

    LOG_write(LOGLEVEL_DEBUG, "[INV] Sending: %d %d %d %d %d %d %d %d",
        msg.data[0], msg.data[1], msg.data[2], msg.data[3],
        msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

    CAN_send(&msg, &hcan1);
}

float INV_get_motor_temp(INV_SideTypeDef side) {
    
}
