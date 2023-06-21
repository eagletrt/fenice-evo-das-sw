#ifndef __INVERTERS_H
#define __INVERTERS_H

#include "stdint.h"

typedef enum {
    INV_LEFT,
    INV_RIGHT
} INV_SideTypeDef;


void INV_parse_CAN_msg(CAN_IdTypeDef id, uint8_t *buf, uint8_t len);

void INV_read_next_register();

float INV_get_IGBT_temp(INV_SideTypeDef side);
float INV_get_motor_temp(INV_SideTypeDef side);
int16_t INV_get_RPM(INV_SideTypeDef side);
bool INV_is_drive_enabled(INV_SideTypeDef side);
bool INV_get_RFE_state(INV_SideTypeDef side);
bool INV_get_FRG_state(INV_SideTypeDef side);

#endif