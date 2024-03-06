#include "race_control.h"

#include "../Lib/can/lib/primary/primary_watchdog.h"
#include "../Lib/can/lib/secondary/secondary_watchdog.h"
#include "can_messages.h"
#include "encoders.h"
#include "inverters.h"
#include "logger.h"
#include "pedals.h"
#include "regen_control.h"
// #include "traction_control.h"

/* If true, the brake has been pressed while APPS was > 25% and
torque needs to stay at 0 Nm until APPS is <= 5% */
bool _DAS_is_brake_impl_on = false;

float _DAS_get_driver_request();
void _DAS_update_brake_impl(float apps, float bse);
bool _DAS_is_control_feasible();

bool equal_d(float a, float b) { return fabs(a - b) < 0.0001; }
bool equal_d_threshold(float a, float b, double threshold)
{
  return fabs(a - b) < threshold;
}

bool _DAS_is_control_feasible()
{
  static uint8_t conditions_counter = 0;
  // Avoid overflow
  if (conditions_counter > CONTROL_FAIL_COUNT)
  {
    conditions_counter = CONTROL_FAIL_COUNT;
  }

  // Driver turned off the controls
  if (equal_d_threshold(CANMSG_SteerStatus.data.map_tv, 0.0, 0.05) &&
      equal_d_threshold(CANMSG_SteerStatus.data.map_sc, 0.0, 0.05))
  {
    conditions_counter = CONTROL_FAIL_COUNT;
    return false;
  }

  // Check on control watchdog
  if (HAL_GetTick() - CANMSG_CtrlOut.info.timestamp >
      PRIMARY_INTERVAL_CONTROL_OUTPUT * 3)
  {
    conditions_counter++;
  }
  if (HAL_GetTick() - CANMSG_CtrlState.info.timestamp >
      SECONDARY_INTERVAL_CONTROL_STATE * 3)
  {
    conditions_counter++;
  }
  else
  {
    // Check control and steering maps
    if (equal_d_threshold(CANMSG_CtrlState.data.map_pw,
                          CANMSG_SteerStatus.data.map_pw, 0.05) &&
        equal_d_threshold(CANMSG_CtrlState.data.map_tv,
                          CANMSG_SteerStatus.data.map_tv, 0.05) &&
        equal_d_threshold(CANMSG_CtrlState.data.map_sc,
                          CANMSG_SteerStatus.data.map_sc, 0.05))
    {
      conditions_counter = 0;
    }
    else
    {
      conditions_counter++;
    }
  }
  return (conditions_counter < CONTROL_FAIL_COUNT) ? true : false;
}

double _DAS_get_speed_ref()
{
  return CANMSG_SteerStatus.data.map_sc;
}
double _DAS_get_torque_ref()
{
  return -CANMSG_SteerStatus.data.map_tv;
}

void DAS_do_drive_routine()
{
  float torque_l_Nm, torque_r_Nm;

  // Check on control watchdog, control power maps and control torque vectoring.
  if (ENABLE_CONTROLS && _DAS_is_control_feasible())
  {
    // if (ENABLE_CONTROLS) {
    torque_l_Nm = CANMSG_CtrlOut.data.torque_l;
    torque_r_Nm = CANMSG_CtrlOut.data.torque_r;
    CANMSG_CarStatus.data.controls_slip = CANMSG_CtrlState.data.map_sc > 0.0f;
    CANMSG_CarStatus.data.controls_torque_vectoring =
        CANMSG_CtrlState.data.map_tv > 0.0f;
  }
  else
  {
    regen_set_speed_ref(&regen_data, _DAS_get_speed_ref());
    regen_set_torque_ref(&regen_data, _DAS_get_torque_ref());
    double regen_torque = regen_get_command(&regen_data);
    if (ENABLE_REGEN && regen_torque < 0.0f)
    {
      torque_l_Nm = torque_r_Nm = regen_torque;
    }
    else
    {
      torque_l_Nm = torque_r_Nm = _DAS_get_driver_request();
    }
  }
  INV_apply_cutoff(INV_get_RPM(INV_LEFT), INV_get_RPM(INV_RIGHT), &torque_l_Nm,
                   &torque_r_Nm);

  INV_set_torque_Nm(INV_LEFT, torque_l_Nm);
  INV_set_torque_Nm(INV_RIGHT, torque_r_Nm);
}

/**
 * @brief     Return the driver requested torque in Nm after the safety checks
 */
float _DAS_get_driver_request()
{
  float APPS_percent = PED_get_accelerator_percent();
  float BSE_percent = PED_get_brake_bar();

  PED_update_plausibility_check();
  _DAS_update_brake_impl(APPS_percent, BSE_percent);

  // if (_DAS_is_brake_impl_on || PED_errors.implausibility_err)
  //     return 0.0f;
  // else
  return PED_get_accelerator_torque(APPS_percent);
}

/**
 * @brief     Update the variable that indicates if torque should be held to 0
 * Nm based on the values of APPS and BSE
 */
void _DAS_update_brake_impl(float apps, float bse)
{
  if (_DAS_is_brake_impl_on)
  {
    if (bse < BRK_THRESHOLD_LOW && apps < 5.0f)
    {
      _DAS_is_brake_impl_on = false;
    }
  }
  else
  {
    if (apps > 25.0f &&
        bse > BRK_IMPL_THRESHOLD)
    { // TODO: aumentare la threshold
      _DAS_is_brake_impl_on = true;
    }
  }
}

float DAS_get_pwr_map() { return CANMSG_SteerStatus.data.map_pw; }

float DAS_get_sc_map() { return CANMSG_SteerStatus.data.map_sc; }

float DAS_get_tv_map() { return CANMSG_SteerStatus.data.map_tv; }
