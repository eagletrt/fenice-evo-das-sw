#include "can.h"
#include "can_user_functions.h"
#include "can_messages.h"
#include "inverters.h"
#include "logger.h"


inverters_inv_r_send_converted_t _INV_r_send = {0U};
inverters_inv_l_send_converted_t _INV_l_send = {0U};
inverters_inv_r_rcv_converted_t _INV_r_recv;
inverters_inv_l_rcv_converted_t _INV_l_recv;

void _INV_send_to_both(uint8_t*, uint8_t);


/** Note: Only left-side registers are listed. Right-side IDs are equal, but remember to send to both! */
typedef struct {
    uint8_t id;
    INV_SideTypeDef side;
    uint32_t interval_ms;
    uint32_t last_read_ms;
} INV_RegMetadataTypeDef;

INV_RegMetadataTypeDef _INV_READ_REG_QUEUE[] = {
    { INVERTERS_INV_L_SEND_READ_ID_22H_I_CMD_RAMP_CHOICE,          INV_LEFT,  20,  0  * 2}, // questo
    { INVERTERS_INV_R_SEND_READ_ID_22H_I_CMD_RAMP_CHOICE,          INV_RIGHT, 20,  1  * 2}, 
    { INVERTERS_INV_L_SEND_READ_ID_26H_I_CMD_CHOICE,               INV_LEFT,  20,  2  * 2}, // questo
    { INVERTERS_INV_R_SEND_READ_ID_26H_I_CMD_CHOICE,               INV_RIGHT, 20,  3  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_27H_IQ_ACTUAL_CHOICE,           INV_LEFT,  20,  4  * 2}, // questo
    { INVERTERS_INV_R_SEND_READ_ID_27H_IQ_ACTUAL_CHOICE,           INV_RIGHT, 20,  5  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_40H_STATUS_MAP_CHOICE,          INV_LEFT,  20,  6  * 2},
    { INVERTERS_INV_R_SEND_READ_ID_40H_STATUS_MAP_CHOICE,          INV_RIGHT, 20,  7  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_49H_T_MOTOR_CHOICE,             INV_LEFT,  50,  8  * 2}, // questo
    { INVERTERS_INV_R_SEND_READ_ID_49H_T_MOTOR_CHOICE,             INV_RIGHT, 50,  9  * 2},
    { INVERTERS_INV_L_SEND_READ_ID_4AH_T_IGBT_CHOICE,              INV_LEFT,  50,  10 * 2}, // questo
    { INVERTERS_INV_R_SEND_READ_ID_4AH_T_IGBT_CHOICE,              INV_RIGHT, 50,  11 * 2},
    { INVERTERS_INV_L_SEND_READ_ID_51H_KERN_MODE_STATE_CHOICE,     INV_LEFT,  100, 12 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_51H_KERN_MODE_STATE_CHOICE,     INV_RIGHT, 100, 13 * 2},

    { INVERTERS_INV_L_SEND_READ_ID_52H_STATUS_MASK_CHOICE,         INV_LEFT,  100, 14 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_52H_STATUS_MASK_CHOICE,         INV_RIGHT, 100, 15 * 2},

    { INVERTERS_INV_L_SEND_READ_ID_8FH_ERRORWARNING_MAP_CHOICE,    INV_LEFT,  100, 16 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_8FH_ERRORWARNING_MAP_CHOICE,    INV_RIGHT, 100, 17 * 2},
    { INVERTERS_INV_L_SEND_READ_ID_A8H_N_ACTUAL_FILT_CHOICE,       INV_LEFT,  20,  18 * 2}, // questo
    { INVERTERS_INV_R_SEND_READ_ID_A8H_N_ACTUAL_FILT_CHOICE,       INV_RIGHT, 20,  19 * 2},
    { INVERTERS_INV_L_SEND_READ_ID_D8H_LOGICREADBITSIN_OUT_CHOICE, INV_LEFT,  20,  20 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_D8H_LOGICREADBITSIN_OUT_CHOICE, INV_RIGHT, 20,  21 * 2},
    { INVERTERS_INV_L_SEND_READ_ID_C0H_DEF_DIN_1_CHOICE,           INV_LEFT,  100, 22 * 2},
    { INVERTERS_INV_R_SEND_READ_ID_C0H_DEF_DIN_1_CHOICE,           INV_RIGHT, 100, 23 * 2},

    { INVERTERS_INV_L_SEND_READ_ID_3AH_M_CMD_RAMP_CHOICE,          INV_LEFT,  20,  24 * 2}, // questo
    { INVERTERS_INV_R_SEND_READ_ID_3AH_M_CMD_RAMP_CHOICE,          INV_RIGHT, 20,  25 * 2},
    { INVERTERS_INV_L_SEND_READ_ID_EBH_VDC_BUS_CHOICE,             INV_RIGHT, 20,  26 * 2}, // questo
    { INVERTERS_INV_R_SEND_READ_ID_EBH_VDC_BUS_CHOICE,             INV_RIGHT, 20,  27 * 2},
};
uint8_t _INV_READ_REG_QUEUE_LEN = sizeof(_INV_READ_REG_QUEUE) / sizeof(_INV_READ_REG_QUEUE[0]);
uint8_t _INV_last_read_reg_idx = 0;


/**
 * @brief     Enable the periodic status update of a register's content based on interval
 */
void INV_enable_regid_updates(uint16_t regid, uint8_t interval) {
    uint8_t buf[] = {INV_CMD_TX_REQ, regid, interval };
    _INV_send_to_both(buf, 3);
}

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
        inverters_inv_l_rcv_unpack(&tmp, buf, INVERTERS_INV_L_RCV_BYTE_SIZE);
        inverters_inv_l_rcv_raw_to_conversion_struct(&_INV_l_recv, &tmp);
        
        CANMSG_MetadataTypeDef *info;
        info = CANMSG_get_InvL_metadata_from_mux_id(_INV_l_recv.rcv_mux);
        if(info != NULL)
            info->timestamp = HAL_GetTick();
        
    } else if (id == INVERTERS_INV_R_RCV_FRAME_ID) {
        inverters_inv_r_rcv_t tmp;
        inverters_inv_r_rcv_unpack(&tmp, buf, INVERTERS_INV_R_RCV_BYTE_SIZE);
        inverters_inv_r_rcv_raw_to_conversion_struct(&_INV_r_recv, &tmp);

        CANMSG_MetadataTypeDef *info;
        info = CANMSG_get_InvR_metadata_from_mux_id(_INV_r_recv.rcv_mux);
        if(info != NULL)
            info->timestamp = HAL_GetTick();
    }
    INV_fill_struct(id, &_INV_r_recv, &_INV_l_recv);
}

void INV_fill_struct(CAN_IdTypeDef id, inverters_inv_r_rcv_converted_t *INV_r_recv, inverters_inv_l_rcv_converted_t *INV_l_recv) {
    static inverters_inv_r_rcv_converted_t INV_r_filled;
    static inverters_inv_l_rcv_converted_t INV_l_filled;
    if (id == INVERTERS_INV_L_RCV_FRAME_ID) {
        INV_l_filled.rcv_mux = INV_l_recv->rcv_mux;
        switch(INV_l_filled.rcv_mux) {
            case INVERTERS_INV_L_RCV_RCV_MUX_ID_27_IQ_ACTUAL_CHOICE:
                INV_l_filled.iq_actual = INV_l_recv->iq_actual;
            break;
            case INVERTERS_INV_L_RCV_RCV_MUX_ID_30_N_ACTUAL_CHOICE:
                INV_l_filled.n_actual = INV_l_recv->n_actual;
            break;
            case INVERTERS_INV_L_RCV_RCV_MUX_ID_40_STATUS_MAP_CHOICE:
                INV_l_filled.ena64 = INV_l_recv->ena64;
                INV_l_filled.ncr064 = INV_l_recv->ncr064;
                INV_l_filled.lim_plus64 = INV_l_recv->lim_plus64;
                INV_l_filled.lim_minus64 = INV_l_recv->lim_minus64;
                INV_l_filled.ok64 = INV_l_recv->ok64;
                INV_l_filled.icns64 = INV_l_recv->icns64;
                INV_l_filled.tnlim64 = INV_l_recv->tnlim64;
                INV_l_filled.pn64 = INV_l_recv->pn64;
                INV_l_filled.ni64 = INV_l_recv->ni64;
                INV_l_filled._n064 = INV_l_recv->_n064;
                INV_l_filled.rsw64 = INV_l_recv->rsw64;
                INV_l_filled.cal064 = INV_l_recv->cal064;
                INV_l_filled.cal64 = INV_l_recv->cal64;
                INV_l_filled.tol64 = INV_l_recv->tol64;
                INV_l_filled.rdy64 = INV_l_recv->rdy64;
                INV_l_filled.brk064 = INV_l_recv->brk064;
                INV_l_filled.signmag64 = INV_l_recv->signmag64;
                INV_l_filled.nclip64 = INV_l_recv->nclip64;
                INV_l_filled.nclip_minus64 = INV_l_recv->nclip_minus64;
                INV_l_filled.nclip_plus64 = INV_l_recv->nclip_plus64;
                INV_l_filled.irddig64 = INV_l_recv->irddig64;
                INV_l_filled.iuserchd64 = INV_l_recv->iuserchd64;
                INV_l_filled.irdn64 = INV_l_recv->irdn64;
                INV_l_filled.irdti64 = INV_l_recv->irdti64;
                INV_l_filled.irdtir64 = INV_l_recv->irdtir64;
                INV_l_filled._10hz64 = INV_l_recv->_10hz64;
                INV_l_filled.irdtm64 = INV_l_recv->irdtm64;
                INV_l_filled.irdana64 = INV_l_recv->irdana64;
                INV_l_filled.iwcns64 = INV_l_recv->iwcns64;
                INV_l_filled.rfepulse64 = INV_l_recv->rfepulse64;
                INV_l_filled.md64 = INV_l_recv->md64;
                INV_l_filled.hndwhl64 = INV_l_recv->hndwhl64;
            break;
            case INVERTERS_INV_L_RCV_RCV_MUX_ID_4A_T_IGBT_CHOICE:
                INV_l_filled.t_igbt = INV_l_recv->t_igbt;
            break;
            case INVERTERS_INV_L_RCV_RCV_MUX_ID_A8_N_ACTUAL_FILT_CHOICE:
                INV_l_filled.n_actual_filt = INV_l_recv->n_actual_filt;
            break;
            case INVERTERS_INV_L_RCV_RCV_MUX_ID_51_KERN_MODE_STATE_CHOICE:
                INV_l_filled.km_rsvd_0 = INV_l_recv->km_rsvd_0;
                INV_l_filled.km_speed_0 = INV_l_recv->km_speed_0;
                INV_l_filled.km_frg_off = INV_l_recv->km_frg_off;
                INV_l_filled.km_cal_off = INV_l_recv->km_cal_off;
                INV_l_filled.km_tx_tog_stat = INV_l_recv->km_tx_tog_stat;
                INV_l_filled.km_i_limit = INV_l_recv->km_i_limit;
                INV_l_filled.km_n_clip = INV_l_recv->km_n_clip;
                INV_l_filled.km_mix_ana_on = INV_l_recv->km_mix_ana_on;
                INV_l_filled.km_allow_sync = INV_l_recv->km_allow_sync;
                INV_l_filled.km_handwheel = INV_l_recv->km_handwheel;
                INV_l_filled.km_phasing_extend = INV_l_recv->km_phasing_extend;
                INV_l_filled.km_rsvd_11 = INV_l_recv->km_rsvd_11;
                INV_l_filled.km_rsvd_12 = INV_l_recv->km_rsvd_12;
                INV_l_filled.km_rsvd_13 = INV_l_recv->km_rsvd_13;
                INV_l_filled.km_pseudo_enable = INV_l_recv->km_pseudo_enable;
                INV_l_filled.km_debug_test = INV_l_recv->km_debug_test;
            break;
            case INVERTERS_INV_L_RCV_RCV_MUX_ID_49_T_MOTOR_CHOICE:
                INV_l_filled.t_motor = INV_l_recv->t_motor;
            break;
            case INVERTERS_INV_L_RCV_RCV_MUX_ID_D8_LOGICREADBITSIN_OUT_CHOICE:
                INV_l_filled.lmt_active_1 = INV_l_recv->lmt_active_1;
                INV_l_filled.lmt_active_2 = INV_l_recv->lmt_active_2;
                INV_l_filled.in_active_2 = INV_l_recv->in_active_2;
                INV_l_filled.in_active_1 = INV_l_recv->in_active_1;
                INV_l_filled.frgrun = INV_l_recv->frgrun;
                INV_l_filled.rfe216 = INV_l_recv->rfe216;
                INV_l_filled.d_out_1_on = INV_l_recv->d_out_1_on;
                INV_l_filled.d_out_2_on = INV_l_recv->d_out_2_on;
                INV_l_filled.btbrdy = INV_l_recv->btbrdy;
                INV_l_filled.go216 = INV_l_recv->go216;
                INV_l_filled.d_out_3_on = INV_l_recv->d_out_3_on;
                INV_l_filled.d_out_4_on = INV_l_recv->d_out_4_on;
                INV_l_filled.goff = INV_l_recv->goff;
                INV_l_filled.brk1216 = INV_l_recv->brk1216;
            break;
            default:
            break;
        }
    } else if (id == INVERTERS_INV_R_RCV_FRAME_ID) {
        INV_r_filled.rcv_mux = INV_r_recv->rcv_mux;
        switch(INV_r_filled.rcv_mux) {
            case INVERTERS_INV_R_RCV_RCV_MUX_ID_27_IQ_ACTUAL_CHOICE:
                INV_r_filled.iq_actual = INV_r_recv->iq_actual;
            break;
            case INVERTERS_INV_R_RCV_RCV_MUX_ID_30_N_ACTUAL_CHOICE:
                INV_r_filled.n_actual = INV_r_recv->n_actual;
            break;
            case INVERTERS_INV_R_RCV_RCV_MUX_ID_40_STATUS_MAP_CHOICE:
                INV_r_filled.ena64 = INV_r_recv->ena64;
                INV_r_filled.ncr064 = INV_r_recv->ncr064;
                INV_r_filled.lim_plus64 = INV_r_recv->lim_plus64;
                INV_r_filled.lim_minus64 = INV_r_recv->lim_minus64;
                INV_r_filled.ok64 = INV_r_recv->ok64;
                INV_r_filled.icns64 = INV_r_recv->icns64;
                INV_r_filled.tnlim64 = INV_r_recv->tnlim64;
                INV_r_filled.pn64 = INV_r_recv->pn64;
                INV_r_filled.ni64 = INV_r_recv->ni64;
                INV_r_filled._n064 = INV_r_recv->_n064;
                INV_r_filled.rsw64 = INV_r_recv->rsw64;
                INV_r_filled.cal064 = INV_r_recv->cal064;
                INV_r_filled.cal64 = INV_r_recv->cal64;
                INV_r_filled.tol64 = INV_r_recv->tol64;
                INV_r_filled.rdy64 = INV_r_recv->rdy64;
                INV_r_filled.brk064 = INV_r_recv->brk064;
                INV_r_filled.signmag64 = INV_r_recv->signmag64;
                INV_r_filled.nclip64 = INV_r_recv->nclip64;
                INV_r_filled.nclip_minus64 = INV_r_recv->nclip_minus64;
                INV_r_filled.nclip_plus64 = INV_r_recv->nclip_plus64;
                INV_r_filled.irddig64 = INV_r_recv->irddig64;
                INV_r_filled.iuserchd64 = INV_r_recv->iuserchd64;
                INV_r_filled.irdn64 = INV_r_recv->irdn64;
                INV_r_filled.irdti64 = INV_r_recv->irdti64;
                INV_r_filled.irdtir64 = INV_r_recv->irdtir64;
                INV_r_filled._10hz64 = INV_r_recv->_10hz64;
                INV_r_filled.irdtm64 = INV_r_recv->irdtm64;
                INV_r_filled.irdana64 = INV_r_recv->irdana64;
                INV_r_filled.iwcns64 = INV_r_recv->iwcns64;
                INV_r_filled.rfepulse64 = INV_r_recv->rfepulse64;
                INV_r_filled.md64 = INV_r_recv->md64;
                INV_r_filled.hndwhl64 = INV_r_recv->hndwhl64;
            break;
            case INVERTERS_INV_R_RCV_RCV_MUX_ID_4A_T_IGBT_CHOICE:
                INV_r_filled.t_igbt = INV_r_recv->t_igbt;
            break;
            case INVERTERS_INV_R_RCV_RCV_MUX_ID_A8_N_ACTUAL_FILT_CHOICE:
                INV_r_filled.n_actual_filt = INV_r_recv->n_actual_filt;
            break;
            case INVERTERS_INV_R_RCV_RCV_MUX_ID_51_KERN_MODE_STATE_CHOICE:
                INV_r_filled.km_rsvd_0 = INV_r_recv->km_rsvd_0;
                INV_r_filled.km_speed_0 = INV_r_recv->km_speed_0;
                INV_r_filled.km_frg_off = INV_r_recv->km_frg_off;
                INV_r_filled.km_cal_off = INV_r_recv->km_cal_off;
                INV_r_filled.km_tx_tog_stat = INV_r_recv->km_tx_tog_stat;
                INV_r_filled.km_i_limit = INV_r_recv->km_i_limit;
                INV_r_filled.km_n_clip = INV_r_recv->km_n_clip;
                INV_r_filled.km_mix_ana_on = INV_r_recv->km_mix_ana_on;
                INV_r_filled.km_allow_sync = INV_r_recv->km_allow_sync;
                INV_r_filled.km_handwheel = INV_r_recv->km_handwheel;
                INV_r_filled.km_phasing_extend = INV_r_recv->km_phasing_extend;
                INV_r_filled.km_rsvd_11 = INV_r_recv->km_rsvd_11;
                INV_r_filled.km_rsvd_12 = INV_r_recv->km_rsvd_12;
                INV_r_filled.km_rsvd_13 = INV_r_recv->km_rsvd_13;
                INV_r_filled.km_pseudo_enable = INV_r_recv->km_pseudo_enable;
                INV_r_filled.km_debug_test = INV_r_recv->km_debug_test;
            break;
            case INVERTERS_INV_R_RCV_RCV_MUX_ID_49_T_MOTOR_CHOICE:
                INV_r_filled.t_motor = INV_r_recv->t_motor;
            break;
            case INVERTERS_INV_R_RCV_RCV_MUX_ID_D8_LOGICREADBITSIN_OUT_CHOICE:
                INV_r_filled.lmt_active_1 = INV_r_recv->lmt_active_1;
                INV_r_filled.lmt_active_2 = INV_r_recv->lmt_active_2;
                INV_r_filled.in_active_2 = INV_r_recv->in_active_2;
                INV_r_filled.in_active_1 = INV_r_recv->in_active_1;
                INV_r_filled.frgrun = INV_r_recv->frgrun;
                INV_r_filled.rfe216 = INV_r_recv->rfe216;
                INV_r_filled.d_out_1_on = INV_r_recv->d_out_1_on;
                INV_r_filled.d_out_2_on = INV_r_recv->d_out_2_on;
                INV_r_filled.btbrdy = INV_r_recv->btbrdy;
                INV_r_filled.go216 = INV_r_recv->go216;
                INV_r_filled.d_out_3_on = INV_r_recv->d_out_3_on;
                INV_r_filled.d_out_4_on = INV_r_recv->d_out_4_on;
                INV_r_filled.goff = INV_r_recv->goff;
                INV_r_filled.brk1216 = INV_r_recv->brk1216;
            break;
            default:
            break;
        }
    }
    *INV_r_recv = INV_r_filled;
    *INV_l_recv = INV_l_filled;
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
    if (rpm < INV_CUTOFF_RPM)
        return request;
        
    float lim_Nm = INV_CUTOFF_COEFF_TORQUE / rpm;

    return (request > lim_Nm && lim_Nm > 0) ? lim_Nm : request;
}

int16_t INV_current_to_num(float current) {
    /* Check Inverter datasheet */
    //A maximum value of 32.768 equals to 169.9A max absorbed
    float num = (current / INV_CURR_PEAK_REAL) * INT16_MAX;
    return (int16_t)clamp(num, INT16_MIN + 1, INT16_MAX - 1);
}

float INV_get_IGBT_temp(INV_SideTypeDef side) {
    uint16_t raw_temp = (side == INV_LEFT) ? _INV_l_recv.t_igbt : _INV_r_recv.t_igbt;
    return -43.23745 + 0.01073427 * raw_temp - 5.523417e-7 * pow(raw_temp, 2) +
         1.330787e-11 * pow(raw_temp, 3);
}

float INV_get_motor_temp(INV_SideTypeDef side) {
    uint16_t raw_temp = (side == INV_LEFT) ? _INV_l_recv.t_motor : _INV_r_recv.t_motor;
    return (raw_temp - 9393.9f) / 55.1f;
}

float INV_get_RPM(INV_SideTypeDef side) {
<<<<<<< HEAD
    float raw_rpm = (side == INV_LEFT) ? -_INV_l_recv.n_actual : _INV_r_recv.n_actual;
    return (float)(raw_rpm*6500/3600); // TODO: Probably needs conversion!
=======
    float raw_rpm = (side == INV_LEFT) ? - _INV_l_recv.n_actual_filt : _INV_r_recv.n_actual_filt;
    raw_rpm *= 10.0f; // DBC conversion backward
    return raw_rpm * (MOT_RPM_LIMIT_REAL / INT16_MAX);
>>>>>>> fix-cutoff
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

bool INV_check_settings() {
    bool retval =  _INV_l_recv.def_end_1 == INVERTERS_INV_L_RCV_DEF_END_1_N_CMD_REVERSE_CHOICE &&
                   _INV_r_recv.def_end_1 == INVERTERS_INV_R_RCV_DEF_END_1_N_CMD_REVERSE_CHOICE;

    retval = retval && _INV_l_recv.active190 == inverters_inv_l_rcv_active190_Low &&
                       _INV_r_recv.active190 == inverters_inv_r_rcv_active190_High;
    return retval;
}

void _INV_send_to_both(uint8_t *buf, uint8_t len) {
    CAN_MessageTypeDef msg;
    msg.size = len;

    for (uint8_t i = 0; i < len; i++)
        msg.data[i] = buf[i];

    msg.id = INV_L_RX_ID;
    CAN_send(&msg, &CAN_PRIMARY_NETWORK);

    msg.id = INV_R_RX_ID;
    CAN_send(&msg, &CAN_PRIMARY_NETWORK);
}