#include "time_constraints.h"

#include "../Lib/can/lib/primary/primary_network.h"
#include "../Lib/can/lib/primary/primary_watchdog.h"
#include "can.h"
#include "can_messages.h"
#include "logger.h"

primary_watchdog _WDG_CAN_watchdog;
CAN_IdTypeDef _WDG_CAN_ids_to_watch[] =
    {PRIMARY_ECU_SET_POWER_MAPS_FRAME_ID, PRIMARY_HV_STATUS_FRAME_ID, PRIMARY_INV_L_RESPONSE_FRAME_ID, PRIMARY_INV_R_RESPONSE_FRAME_ID};

void WDG_init() {
    for (uint8_t i = 0; i < sizeof(_WDG_CAN_ids_to_watch) / sizeof(_WDG_CAN_ids_to_watch[0]); i++) {
        CANLIB_BITSET_ARRAY(_WDG_CAN_watchdog.activated, primary_watchdog_index_from_id(_WDG_CAN_ids_to_watch[i]));
    }
}

void WDG_update_and_check_timestamps() {
    /* Update timestamps */
    for (uint8_t i = 0; i < sizeof(_WDG_CAN_ids_to_watch) / sizeof(_WDG_CAN_ids_to_watch[0]); i++) {
        uint32_t time_ms = CANMSG_get_primary_metadata_from_id(_WDG_CAN_ids_to_watch[i])->timestamp;

        primary_watchdog_reset(&_WDG_CAN_watchdog, _WDG_CAN_ids_to_watch[i], time_ms);
    }

    primary_watchdog_timeout(&_WDG_CAN_watchdog, HAL_GetTick());

    /* Check timings */
    for (int i = 0; i < sizeof(_WDG_CAN_ids_to_watch) / sizeof(_WDG_CAN_ids_to_watch[0]); i++) {
        bool timed_out = CANLIB_BITTEST_ARRAY(_WDG_CAN_watchdog.timeout, primary_watchdog_index_from_id(_WDG_CAN_ids_to_watch[i]));

        if (!timed_out)
            continue;

        switch (_WDG_CAN_ids_to_watch[i]) {
            case PRIMARY_HV_STATUS_FRAME_ID:
                ecumsg_ecu_errors_state.data.error_ts_tout = 1;
                break;
            case PRIMARY_INV_L_RESPONSE_FRAME_ID:
                ecumsg_ecu_errors_state.data.error_invl_tout = 1;
                break;
            case PRIMARY_INV_R_RESPONSE_FRAME_ID:
                ecumsg_ecu_errors_state.data.error_invr_tout = 1;
                break;
            case PRIMARY_ECU_SET_POWER_MAPS_FRAME_ID:
                ecumsg_ecu_errors_state.data.error_steer_tout = 1;
                break;
            default:
                LOG_write(LOGLEVEL_WARN, "[WDG] Timeout for unknown CAN message id 0x%02X", _WDG_CAN_ids_to_watch[i]);
                break;
        }
    }
}

bool WDG_is_car_in_safe_state() {
    WDG_update_and_check_timestamps(HAL_GetTick());

    bool critical_error = false;
    critical_error |= ecumsg_ecu_errors_state.data.error_ts_tout;
    critical_error |= ecumsg_ecu_errors_state.data.error_invl_tout;
    // critical_error |= ecumsg_ecu_errors_state.data.das_error_invr_tout;
    // critical_error |= ecumsg_ecu_errors_state.data.das_error_steer_tout;
    critical_error |= ecumsg_ecu_errors_state.data.error_pedal_adc;
    critical_error |= ecumsg_ecu_errors_state.data.error_pedal_implausibility;

    return (!critical_error);
}
