#ifndef __INVERTERS_H
#define __INVERTERS_H

#include "stdint.h"
#include "can.h"
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265f
#endif

typedef enum {
    INV_LEFT,
    INV_RIGHT
} INV_SideTypeDef;

/* Calculate the correct coefficient to limit motor torque at high RPMs */
#define INV_POWER_LIMIT             40e3 // W
#define MOT_TORQUE_COEFF            0.45 // Nm/Arms // 0,54 sul nuovo datasheet
#define MOT_RPM_LIMIT_REAL          6500.f // #rot/min
#define INV_CUTOFF_COEFF_REAL       INV_POWER_LIMIT*60/(MOT_TORQUE_COEFF*2*M_PI) // A/s
#define INV_CURR_PEAK_REAL          169.9f // A
#define INV_RPM_CUTOFF              INV_CUTOFF_COEFF_REAL/INV_CURR_PEAK_REAL+1 // 1/s
#define INV_CUTOFF_COEFF            (int32_t)(INV_CUTOFF_COEFF_REAL/INV_CURR_PEAK_REAL*INT16_MAX) // 1/s
#define INV_CUTOFF_COEFF_TORQUE     MOT_TORQUE_COEFF / INV_CUTOFF_COEFF_REAL // 1/s


void INV_parse_CAN_msg(CAN_IdTypeDef id, uint8_t *buf, uint8_t len);

void INV_read_next_register();

float INV_torque_to_current(float torque);
int16_t INV_current_to_num(float current);
float INV_cutoff_torque(float request, float rpm);

float INV_get_IGBT_temp(INV_SideTypeDef side);
float INV_get_motor_temp(INV_SideTypeDef side);
int16_t INV_get_RPM(INV_SideTypeDef side);
bool INV_is_drive_enabled(INV_SideTypeDef side);
bool INV_get_RFE_state(INV_SideTypeDef side);
bool INV_get_FRG_state(INV_SideTypeDef side);

void INV_enable_drive(INV_SideTypeDef side);
void INV_disable_drive(INV_SideTypeDef side);
void INV_set_torque_Nm(INV_SideTypeDef side, float torque);

bool INV_check_settings();

#endif