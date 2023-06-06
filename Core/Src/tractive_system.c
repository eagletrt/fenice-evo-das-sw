#include "tractive_system.h"
#include "can_messages.h"
#include "logger.h"


TS_StatusTypeDef _TS_status = TS_STATUS_UNKNOWN;
char* TS_state_names[] = { "UNK", "OFF", "PRE", "ON", "FAIL" };


/**
 * @brief     Retrieve the status of the BMS-HV from the corresponding CAN message
 */
TS_StatusTypeDef TS_get_status() {
    if (CANMSG_TSStatus.info.is_new) {
        CANMSG_TSStatus.info.is_new = false;

        switch (CANMSG_TSStatus.data.ts_status) {
            case primary_ts_status_ts_status_FATAL:
                _TS_status = TS_STATUS_FATAL;
                break;
            case primary_ts_status_ts_status_ON:
                _TS_status = TS_STATUS_ON;
                break;
            case primary_ts_status_ts_status_OFF:
                _TS_status = TS_STATUS_OFF;
                break;
            case primary_ts_status_ts_status_PRECHARGE:
                _TS_status = TS_STATUS_PRECHARGE;
                break;
        }
    }

    // if ((HAL_GetTick() - CANMSG_TSStatus.info.timestamp) > primary_watchdog_interval_from_id(primary_ID_SET_TS_STATUS_DAS))
    //     _TS_status = TS_STATUS_UNKNOWN;

    return _TS_status;
}

/**
 * @brief     Send a TS-ON message to the BMS-HV
 */
void TS_power_on() {
    CANMSG_SetTSStatus.data.ts_status_set = primary_set_ts_status_handcart_ts_status_set_ON;
    CANMSG_SetTSStatus.info.is_new = true;
}

/**
 * @brief     Send a TS-OFF message to the BMS-HV
 */
void TS_power_off() {
    CANMSG_SetTSStatus.data.ts_status_set = primary_set_ts_status_handcart_ts_status_set_OFF;
    CANMSG_SetTSStatus.info.is_new = true;
}
