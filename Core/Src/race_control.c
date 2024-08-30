#include "race_control.h"

#include "../Lib/can/lib/primary/primary_watchdog.h"
#include "../Lib/can/lib/secondary/secondary_watchdog.h"
#include "can_messages.h"
#include "encoders.h"
#include "inverters.h"
#include "logger.h"
#include "pedals.h"
// #include "traction_control.h"

/* If true, the brake has been pressed while APPS was > 25% and
torque needs to stay at 0 Nm until APPS is <= 5% */
bool _DAS_is_brake_impl_on = false;

void _DAS_update_brake_impl(float apps, float bse);

bool equal_d(float a, float b) {
    return fabs(a - b) < 0.0001;
}
bool equal_d_threshold(float a, float b, double threshold) {
    return fabs(a - b) < threshold;
}

bool _DAS_is_control_feasible() {
    // Steering wheel encoder not working
    if ((ENC_C_get_angle_deg()) > 179.0f && (ENC_C_get_angle_deg()) < 181.0f) {
        return false;
    }

    static uint8_t conditions_counter = 0;
    // Avoid overflow
    if (conditions_counter > CONTROL_FAIL_COUNT) {
        conditions_counter = CONTROL_FAIL_COUNT;
    }

    // Driver turned off the controls
    if (!DAS_get_sc_state() && !DAS_get_tv_state() && !DAS_get_reg_state()) {
        conditions_counter                                                    = CONTROL_FAIL_COUNT;
        ecumsg_ecu_control_status_state.data.control_errors_disabled_from_ecu = 1;
        return false;
    }

    // Check on control watchdog
    if (HAL_GetTick() - ecumsg_control_output_state.info.timestamp > PRIMARY_INTERVAL_CONTROL_OUTPUT * 6) {
        conditions_counter++;
        ecumsg_ecu_control_status_state.data.control_errors_control_watchdog = 1;
    }
    if (HAL_GetTick() - ecumsg_control_status_state.info.timestamp > PRIMARY_INTERVAL_CONTROL_STATUS * 6) {
        conditions_counter++;
        ecumsg_ecu_control_status_state.data.control_errors_control_watchdog = 1;
    } else {
        // Check control and steering maps
        if (equal_d_threshold(ecumsg_control_status_state.data.map_power, DAS_get_power_map(), 0.05) &&
            ecumsg_control_status_state.data.sc_state == DAS_get_sc_state() &&
            ecumsg_control_status_state.data.tv_state == DAS_get_tv_state() &&
            ecumsg_control_status_state.data.reg_state == DAS_get_reg_state()) {
            conditions_counter = 0;
        } else {
            conditions_counter++;
            ecumsg_ecu_control_status_state.data.control_errors_wrong_maps = 1;
        }
    }

    if (conditions_counter < CONTROL_FAIL_COUNT) {
        if (conditions_counter == 0) {
            ecumsg_ecu_control_status_state.data.control_errors_disabled_from_ecu = 0;
            ecumsg_ecu_control_status_state.data.control_errors_control_watchdog  = 0;
            ecumsg_ecu_control_status_state.data.control_errors_wrong_maps        = 0;
        }
        return true;
    } else {
        return false;
    }
}

bool DAS_do_drive_routine(float brake_pressure) {
    float torque_l_Nm, torque_r_Nm;

    ecumsg_ecu_control_status_state.info.is_new          = 1;
    ecumsg_ecu_control_status_state.data.control_enabled = false;
    float driver_request                                 = _DAS_get_driver_request();

    // Check on control watchdog, control power maps and control torque vectoring.
    if (ENABLE_CONTROLS && _DAS_is_control_feasible()) {
        ecumsg_ecu_control_status_state.data.control_enabled = true;
        torque_l_Nm                                          = ecumsg_control_output_state.data.torque_l;
        torque_r_Nm                                          = ecumsg_control_output_state.data.torque_r;
    } else {
        torque_l_Nm = torque_r_Nm = driver_request;
    }
    INV_apply_cutoff(INV_get_RPM(INV_LEFT), INV_get_RPM(INV_RIGHT), &torque_l_Nm, &torque_r_Nm);
    bool applied_bspd_limits = INV_apply_bspd_limits(&torque_l_Nm, &torque_r_Nm, brake_pressure);

    INV_set_torque_Nm(INV_LEFT, torque_l_Nm);
    INV_set_torque_Nm(INV_RIGHT, torque_r_Nm);
    return applied_bspd_limits;
}

/**
 * @brief     Return the driver requested torque in Nm after the safety checks
 */
float _DAS_get_driver_request() {
    float APPS_percent = PED_get_accelerator_percent();
    float BSE_percent  = PED_get_brake_bar();

    // PED_update_plausibility_check();
    // if (PED_errors.implausibility_err) {
    // return 0.0f;
    // }
    _DAS_update_brake_impl(APPS_percent, BSE_percent);

    return PED_get_accelerator_torque(APPS_percent);
}

/**
 * @brief     Update the variable that indicates if torque should be held to 0 Nm
 *            based on the values of APPS and BSE
 */
void _DAS_update_brake_impl(float apps, float bse) {
    if (_DAS_is_brake_impl_on) {
        if (bse < BRK_THRESHOLD_LOW && apps < 5.0f) {
            _DAS_is_brake_impl_on = false;
        }
    } else {
        if (apps > 25.0f && bse > BRK_IMPL_THRESHOLD) {  // TODO: aumentare la threshold
            _DAS_is_brake_impl_on = true;
        }
    }
}

float DAS_get_power_map() {
    return 1.0f;
}
bool DAS_get_sc_state() {
    return ecumsg_ecu_set_power_maps_state.data.sc_state;
}
bool DAS_get_tv_state() {
    return ecumsg_ecu_set_power_maps_state.data.tv_state;
}
bool DAS_get_reg_state() {
    return ecumsg_ecu_set_power_maps_state.data.reg_state;
}