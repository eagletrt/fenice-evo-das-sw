#include "race_control.h"

#include "encoders.h"
#include "pedals.h"
#include "inverters.h"
#include "logger.h"
#include "can_messages.h"
// #include "traction_control.h"


/* If true, the brake has been pressed while APPS was > 25% and
torque needs to stay at 0 Nm until APPS is <= 5% */
bool _DAS_is_brake_impl_on = false;


float _DAS_get_driver_request();
void _DAS_update_brake_impl(float apps, float bse);


void DAS_do_drive_routine() {
    float torque_l_Nm, torque_r_Nm;
    
    // if (DAS_get_sc_map() == 0 && DAS_get_tv_map() == 0) {
    torque_l_Nm = torque_r_Nm = _DAS_get_driver_request();
    torque_l_Nm = INV_cutoff_torque(torque_l_Nm, INV_get_RPM(INV_LEFT));
    torque_r_Nm = INV_cutoff_torque(torque_r_Nm, INV_get_RPM(INV_RIGHT));
    // } else {
    //     DAS_sync_CTRL();
    //     float max_torque = 76.455f; /* (Max_current * [Nm/A]) = (169.9 * 0.45) */
    //     torque_l_Nm = CTRL_get_torque_L() / max_torque * 100.0f;
    //     torque_r_Nm = CTRL_get_torque_R() / max_torque * 100.0f;
        
    //     CANMSG_CtrlOut.data.torque_l = CTRL_get_torque_L();
    //     CANMSG_CtrlOut.data.torque_r = CTRL_get_torque_R();
    //     CANMSG_CtrlOut.data.tmax_l = CTRL_get_tmax_L();
    //     CANMSG_CtrlOut.data.tmax_r = CTRL_get_tmax_R();
    //     CANMSG_CtrlOut.data.estimated_velocity = CTRL_get_vest();
    //     CANMSG_CtrlOut.info.is_new = true;
    // }

    INV_set_torque_Nm(INV_LEFT, torque_l_Nm);
    // INV_set_torque_Nm(INV_RIGHT, torque_r_Nm);
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
        if (apps > 25.0f && bse > BRK_DRIVE_THRESHOLD){ // TODO: aumentare la threshold
            _DAS_is_brake_impl_on = true;
        }
    }
}

float DAS_get_pwr_map() {
    return CANMSG_SteerStatus.data.map_pw;
}

float DAS_get_sc_map() {
    return CANMSG_SteerStatus.data.map_sc;
}

float DAS_get_tv_map() {
    return CANMSG_SteerStatus.data.map_tv;
}
