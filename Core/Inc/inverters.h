#ifndef __INVERTERS_H
#define __INVERTERS_H

#include "stdint.h"
#include "can.h"
#include "math.h"
#include "../Lib/can/lib/inverters/inverters_network.h"

#ifndef M_PI
#define M_PI 3.14159265f
#endif

#define INV_L_RX_ID    PRIMARY_ID_INV_L_REQUEST  /*< ID on which the left inverter listens */
#define INV_L_TX_ID    PRIMARY_ID_INV_L_RESPONSE /*< ID with which the left inverter transmits */
#define INV_R_RX_ID    PRIMARY_ID_INV_R_REQUEST  /*< ID on which the right inverter listens */
#define INV_R_TX_ID    PRIMARY_ID_INV_R_RESPONSE /*< ID with which the right inverter transmits */

typedef enum {
    INV_LEFT,
    INV_RIGHT
} INV_SideTypeDef;

/* Register IDs for which updates will be activated */
#define INV_CMD_TX_REQ     0x3D

/* Calculate the correct coefficient to limit motor torque at high RPMs */
#define MOT_RPM_LIMIT_REAL          6500.f // #rot/min
#define INV_POWER_LIMIT             30e3f   // W
#define MOT_TORQUE_COEFF            0.45f  // Nm/Arms // 0,54 sul nuovo datasheet
#define INV_CURR_PEAK_REAL          400.0f // A

#define INV_CUTOFF_COEFF            (INV_POWER_LIMIT/MOT_TORQUE_COEFF * (60.0f/(2*M_PI))) // Arms (Imax)
#define INV_CUTOFF_RPM              (INV_CUTOFF_COEFF / INV_CURR_PEAK_REAL) // 1/s
#define INV_CUTOFF_COEFF_TORQUE     (INV_CUTOFF_COEFF * MOT_TORQUE_COEFF)   // Nm

void INV_enable_regid_updates(uint16_t regid, uint8_t interval);
void INV_parse_CAN_msg(CAN_IdTypeDef id, uint8_t *buf, uint8_t len);
void INV_fill_struct(CAN_IdTypeDef id, inverters_inv_r_rcv_converted_t *INV_r_recv, inverters_inv_l_rcv_converted_t *INV_l_recv);

void INV_read_next_register();

float INV_torque_to_current(float torque);
int16_t INV_current_to_num(float current);
float INV_cutoff_torque(float request, float rpm);

float INV_get_IGBT_temp(INV_SideTypeDef side);
float INV_get_motor_temp(INV_SideTypeDef side);
float INV_get_RPM(INV_SideTypeDef side);
bool INV_is_drive_enabled(INV_SideTypeDef side);
bool INV_get_RFE_state(INV_SideTypeDef side);
bool INV_get_FRG_state(INV_SideTypeDef side);

void INV_enable_drive(INV_SideTypeDef side);
void INV_disable_drive(INV_SideTypeDef side);
void INV_set_torque_Nm(INV_SideTypeDef side, float torque);

bool INV_check_settings();

#endif