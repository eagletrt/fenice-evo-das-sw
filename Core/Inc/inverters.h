#ifndef __INVERTERS_H
#define __INVERTERS_H

#include "../Lib/can/lib/inverters/inverters_network.h"
#include "can.h"
#include "math.h"
#include "stdint.h"

#ifndef M_PI
#define M_PI 3.14159265f
#endif

#define INV_L_RX_ID PRIMARY_ID_INV_L_REQUEST  /*< ID on which the left inverter listens */
#define INV_L_TX_ID PRIMARY_ID_INV_L_RESPONSE /*< ID with which the left inverter transmits */
#define INV_R_RX_ID PRIMARY_ID_INV_R_REQUEST  /*< ID on which the right inverter listens */
#define INV_R_TX_ID PRIMARY_ID_INV_R_RESPONSE /*< ID with which the right inverter transmits */

typedef enum { INV_LEFT, INV_RIGHT } INV_SideTypeDef;

/* Register IDs for which updates will be activated */
#define INV_CMD_TX_REQ 0x3D

#define HV_MAX_REGEN_CURRENT      -24.0f
#define HV_MIN_CELL_VOLTAGE       2.8f
#define HV_CELL_COUNT             144
#define P_BAT_MAX                 (80 * 1e3)  // W
#define I_BAT_MAX                 140.0f
#define BAT_PARALLELS             3.0f
#define P_BAT_MIN                 (HV_MAX_REGEN_CURRENT * HV_MIN_CELL_VOLTAGE * HV_CELL_COUNT)
#define P_MOT_MAX                 60 * 1e3  // W
#define MOT_TORQUE_PEAK           100.0f    // Nm
#define MOT_TORQUE_COEFF          0.54f     // Nm/Arms
#define RPM_TO_RADS_COEFF         (2 * M_PI / 60.0f)
#define RADS_TO_RPM_COEFF         (60.0f / (2 * M_PI))
#define INV_I_MAX                 169.7f     //  400.0f // Arms
#define MOT_RPM_MAX               8000.0f    // RPM
#define BSPD_BRAKE_PRESSURE_LIMIT (20.0f)    // Bar
#define BSPD_POWER_LIMIT          (3800.0f)  // W

float INV_I_mot_peak();
float INV_I_mot_max(const float rpm);
float INV_I_batt_max(const float rpm, const float torque_ratio);
void INV_apply_cutoff(const float rpm_l, const float rpm_r, float *torque_l, float *torque_r);
bool INV_apply_bspd_limits(float *torque_l_Nm, float *torque_r_Nm, float brake_pressure);

void INV_enable_regid_updates(uint16_t regid, uint8_t interval);
void INV_parse_CAN_msg(CAN_IdTypeDef id, uint8_t *buf, uint8_t len);
void INV_fill_struct(CAN_IdTypeDef id, inverters_inv_r_rcv_converted_t *INV_r_recv, inverters_inv_l_rcv_converted_t *INV_l_recv);

void INV_read_next_register();

float INV_torque_to_current(float torque);
float INV_current_to_torque(float current);
int16_t INV_current_to_num(float current);

float INV_get_IGBT_temp(INV_SideTypeDef side);
float INV_get_motor_temp(INV_SideTypeDef side);
float INV_get_RPM(INV_SideTypeDef side);
bool INV_is_drive_enabled(INV_SideTypeDef side);
bool INV_get_RFE_state(INV_SideTypeDef side);
bool INV_get_FRG_state(INV_SideTypeDef side);

void _INV_minimum_cell_voltage_limit(float rpm_l, float rpm_r, float *torque_l, float *torque_r);

void INV_enable_drive(INV_SideTypeDef side);
void INV_disable_drive(INV_SideTypeDef side);
void INV_set_torque_Nm(INV_SideTypeDef side, float torque);

bool INV_check_settings();

#endif