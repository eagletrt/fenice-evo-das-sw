#include "tractive_system.h"
#include "can_messages.h"
#include "logger.h"


TS_StatusTypeDef _TS_status = TS_STATUS_UNKNOWN;
char* TS_state_names[] = { "UNK", "OFF", "PRE", "ON", "FAIL" };


/**
 * @brief     Retrieve the status of the BMS-HV from the corresponding CAN message
 */
TS_StatusTypeDef TS_get_status() {
    if (ecumsg_hv_status_state.info.is_new) {
        ecumsg_hv_status_state.info.is_new = false;

        switch (ecumsg_hv_status_state.data.status) {
            case primary_hv_status_status_fatal_error:
                _TS_status = TS_STATUS_FATAL;
                break;
            case primary_hv_status_status_ts_on:
                _TS_status = TS_STATUS_ON;
                break;
            case primary_hv_status_status_init:
            case primary_hv_status_status_idle:
                _TS_status = TS_STATUS_OFF;
                break;
            case primary_hv_status_status_airn_close:
            case primary_hv_status_status_precharge:
            case primary_hv_status_status_airp_close:
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
    ecumsg_hv_set_status_ecu_state.data.hv_status_set = primary_hv_set_status_ecu_hv_status_set_on;
    ecumsg_hv_set_status_ecu_state.info.is_new = true;
}

/**
 * @brief     Send a TS-OFF message to the BMS-HV
 */
void TS_power_off() {
    ecumsg_hv_set_status_ecu_state.data.hv_status_set = primary_hv_set_status_ecu_hv_status_set_off;
    ecumsg_hv_set_status_ecu_state.info.is_new = true;
}
