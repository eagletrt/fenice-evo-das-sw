#ifndef TRACTIVE_SYSTEM_H
#define TRACTIVE_SYSTEM_H


typedef enum {
    TS_STATUS_UNKNOWN = 0U,     /*< Status not yet known */
    TS_STATUS_OFF,
    TS_STATUS_PRECHARGE,
    TS_STATUS_ON,
    TS_STATUS_FATAL
} TS_StatusTypeDef;

extern char* TS_state_names[5];

TS_StatusTypeDef TS_get_status();
void TS_power_on();
void TS_power_off();

#endif