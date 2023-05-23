#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include "can.h"
#include "stdbool.h"


extern float CAN_error_rate;

bool CAN_user_init(CAN_HandleTypeDef *);
HAL_StatusTypeDef CAN_send(CAN_MessageTypeDef*, CAN_HandleTypeDef*);

#endif