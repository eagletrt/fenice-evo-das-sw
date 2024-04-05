#include "race_control.h"

#include "encoders.h"
#include "pedals.h"
#include "inverters.h"
#include "logger.h"
#include "can_messages.h"
#include "../Lib/can/lib/primary/primary_watchdog.h"
#include "../Lib/can/lib/secondary/secondary_watchdog.h"
// #include "traction_control.h"


/* If true, the brake has been pressed while APPS was > 25% and
torque needs to stay at 0 Nm until APPS is <= 5% */
bool _DAS_is_brake_impl_on = false;


float _DAS_get_driver_request();
void _DAS_update_brake_impl(float apps, float bse);
bool _DAS_is_control_feasible();

bool equal_d(float a, float b) {
    return fabs(a - b) < 0.0001;
}
bool equal_d_threshold(float a, float b, double threshold) {
    return fabs(a - b) < threshold;
}

bool _DAS_is_control_feasible() {
    static uint8_t conditions_counter = 0;
    // Avoid overflow
    if(conditions_counter > CONTROL_FAIL_COUNT) {
        conditions_counter = CONTROL_FAIL_COUNT;
    }
        ecumsg_ecu_control_status_state.data.control_errors_forced_off = 0;

    // Driver turned off the controls
    if (equal_d_threshold(ecumsg_ecu_set_power_maps_state.data.map_tv, 0.0, 0.05) &&
        equal_d_threshold(ecumsg_ecu_set_power_maps_state.data.map_sc, 0.0, 0.05)) {
        conditions_counter = CONTROL_FAIL_COUNT;
        ecumsg_ecu_control_status_state.data.control_errors_forced_off = 1;
        return false;
    }

    // Check on control watchdog
    if(HAL_GetTick() - ecumsg_control_output_state.info.timestamp > PRIMARY_INTERVAL_CONTROL_OUTPUT * 3) {
        conditions_counter ++;
        ecumsg_ecu_control_status_state.data.control_errors_control_watchdog = 1;
    }
    if(HAL_GetTick() - ecumsg_control_status_state.info.timestamp > PRIMARY_INTERVAL_CONTROL_STATUS * 3) {
        conditions_counter ++;
        ecumsg_ecu_control_status_state.data.control_errors_control_watchdog = 1;
    } else {
        // Check control and steering maps
        if (equal_d_threshold(ecumsg_control_status_state.data.map_pw, ecumsg_ecu_set_power_maps_state.data.map_pw, 0.05) && 
            equal_d_threshold(ecumsg_control_status_state.data.map_tv, ecumsg_ecu_set_power_maps_state.data.map_tv, 0.05) && 
            equal_d_threshold(ecumsg_control_status_state.data.map_sc, ecumsg_ecu_set_power_maps_state.data.map_sc, 0.05)) {
            conditions_counter = 0;
        } else {
            conditions_counter ++;
            ecumsg_ecu_control_status_state.data.control_errors_wrong_maps = 1;
        }
    }

    if(conditions_counter < CONTROL_FAIL_COUNT) {
        if(conditions_counter == 0) {
            ecumsg_ecu_control_status_state.data.control_errors_forced_off = 0;
            ecumsg_ecu_control_status_state.data.control_errors_control_watchdog = 0;
            ecumsg_ecu_control_status_state.data.control_errors_wrong_maps = 0;
        }
        return true;
    } else {
        return false;
    }
}

void DAS_do_drive_routine() {
    float torque_l_Nm, torque_r_Nm;

    ecumsg_ecu_control_status_state.data.control_enabled = ENABLE_CONTROLS;
    ecumsg_ecu_control_status_state.info.is_new = 1;
    
    // Check on control watchdog, control power maps and control torque vectoring.
    if(ENABLE_CONTROLS && _DAS_is_control_feasible()) {
        torque_l_Nm = ecumsg_control_output_state.data.torque_l;
        torque_r_Nm = ecumsg_control_output_state.data.torque_r;
    } else {
        torque_l_Nm = torque_r_Nm = _DAS_get_driver_request();
    }
    INV_apply_cutoff(INV_get_RPM(INV_LEFT), INV_get_RPM(INV_RIGHT), &torque_l_Nm, &torque_r_Nm);

    INV_set_torque_Nm(INV_LEFT, torque_l_Nm);
    INV_set_torque_Nm(INV_RIGHT, torque_r_Nm);
}

/**
 * @brief     Return the driver requested torque in Nm after the safety checks
 */
float _DAS_get_driver_request() {
    float APPS_percent = PED_get_accelerator_percent();
    float BSE_percent = PED_get_brake_bar();

    PED_update_plausibility_check();
    _DAS_update_brake_impl(APPS_percent, BSE_percent);

    if (_DAS_is_brake_impl_on || PED_errors.implausibility_err)
        return 0.0f;
    else
        return PED_get_accelerator_torque(APPS_percent);
}

/**
 * @brief     Update the variable that indicates if torque should be held to 0 Nm
 *            based on the values of APPS and BSE
 */
void _DAS_update_brake_impl(float apps, float bse) {
    if (_DAS_is_brake_impl_on) {
        if (bse < BRK_THRESHOLD_LOW && apps < 5.0f){
            _DAS_is_brake_impl_on = false;
        }
    } else {
        if (apps > 25.0f && bse > BRK_IMPL_THRESHOLD){ // TODO: aumentare la threshold
            _DAS_is_brake_impl_on = true;
        }
    }
}

float DAS_get_pwr_map() {
    return ecumsg_ecu_set_power_maps_state.data.map_pw;
}

float DAS_get_sc_map() {
    return ecumsg_ecu_set_power_maps_state.data.map_sc;
}

float DAS_get_tv_map() {
    return ecumsg_ecu_set_power_maps_state.data.map_tv;
}
