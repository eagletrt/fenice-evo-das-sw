#ifndef __INVERTERS_H
#define __INVERTERS_H

#include "stdint.h"

typedef enum {
    INV_LEFT,
    INV_RIGHT
} INV_SideTypeDef;


void INV_parse_CAN_msg(uint8_t *buf, uint8_t len);

void INV_read_next_register();

float INV_get_inv_temp(INV_SideTypeDef side);

#endif