#ifndef TSAC_API_H
#define TSAC_API_H

#include "tsac.h"
#include "can-primary.h"

void tsac_api_init(void);
enum TsacStatus tsac_api_get_status(void);
enum TsacReturnCode tsac_api_set_status(enum TsacStatus status);
enum TsacReturnCode tsac_api_ask_power_on(void);
enum TsacReturnCode tsac_api_ask_power_off(void);
enum TsacStatus tsac_api_convert_from_can_status(enum CanPrimaryHvBmsStatusName can_status);

#endif // TSAC_API_H
