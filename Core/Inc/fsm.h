/******************************************************************************
Finite State Machine
Project: Docs/fsm.dot
Description: vehicle_fsm

Generated by gv_fsm ruby gem, see https://rubygems.org/gems/gv_fsm
gv_fsm version 0.3.4
Generation date: 2024-08-07 11:43:48 +0200
Generated from: Docs/fsm.dot
The finite state machine has:
  14 states
  0 transition functions
******************************************************************************/

#ifndef FSM_H
#define FSM_H

#include "adc_fsm.h"
#include "buzzer.h"
#include "can_messages.h"
#include "inverters.h"
#include "logger.h"
#include "pedals.h"
#include "race_control.h"
#include "time_constraints.h"
#include "tractive_system.h"

#include <stdlib.h>

// State data object
// By default set to void; override this typedef or load the proper
// header if you need
typedef void state_data_t;

// NOTHING SHALL BE CHANGED AFTER THIS LINE!

// List of states
typedef enum {
    STATE_INIT = 0,
    STATE_ENABLE_INV_UPDATES,
    STATE_CHECK_INV_SETTINGS,
    STATE_IDLE,
    STATE_FATAL_ERROR,
    STATE_START_TS_PRECHARGE,
    STATE_WAIT_TS_PRECHARGE,
    STATE_START_TS_DISCHARGE,
    STATE_WAIT_DRIVER,
    STATE_ENABLE_INV_DRIVE,
    STATE_DRIVE,
    STATE_DISABLE_INV_DRIVE,
    STATE_RE_ENABLE_INV_DRIVE,
    STATE_WAIT_TS_DISCHARGE,
    NUM_STATES,
    NO_CHANGE
} state_t;

// State human-readable names
extern const char *state_names[];

// State function prototype
typedef state_t state_func_t(state_data_t *data);

// State functions

// Function to be executed in state init
// valid return states: STATE_ENABLE_INV_UPDATES
state_t do_init(state_data_t *data);

// Function to be executed in state enable_inv_updates
// valid return states: NO_CHANGE, STATE_ENABLE_INV_UPDATES,
// STATE_CHECK_INV_SETTINGS
state_t do_enable_inv_updates(state_data_t *data);

// Function to be executed in state check_inv_settings
// valid return states: NO_CHANGE, STATE_CHECK_INV_SETTINGS, STATE_IDLE,
// STATE_FATAL_ERROR
state_t do_check_inv_settings(state_data_t *data);

// Function to be executed in state idle
// valid return states: NO_CHANGE, STATE_IDLE, STATE_FATAL_ERROR,
// STATE_START_TS_PRECHARGE
state_t do_idle(state_data_t *data);

// Function to be executed in state fatal_error
// valid return states: NO_CHANGE, STATE_FATAL_ERROR
state_t do_fatal_error(state_data_t *data);

// Function to be executed in state start_ts_precharge
// valid return states: NO_CHANGE, STATE_START_TS_PRECHARGE,
// STATE_WAIT_TS_PRECHARGE, STATE_START_TS_DISCHARGE
state_t do_start_ts_precharge(state_data_t *data);

// Function to be executed in state wait_ts_precharge
// valid return states: NO_CHANGE, STATE_WAIT_TS_PRECHARGE, STATE_WAIT_DRIVER,
// STATE_START_TS_DISCHARGE
state_t do_wait_ts_precharge(state_data_t *data);

// Function to be executed in state start_ts_discharge
// valid return states: NO_CHANGE, STATE_START_TS_DISCHARGE,
// STATE_WAIT_TS_DISCHARGE
state_t do_start_ts_discharge(state_data_t *data);

// Function to be executed in state wait_driver
// valid return states: NO_CHANGE, STATE_WAIT_DRIVER, STATE_ENABLE_INV_DRIVE,
// STATE_START_TS_DISCHARGE
state_t do_wait_driver(state_data_t *data);

// Function to be executed in state enable_inv_drive
// valid return states: NO_CHANGE, STATE_ENABLE_INV_DRIVE, STATE_DRIVE,
// STATE_DISABLE_INV_DRIVE
state_t do_enable_inv_drive(state_data_t *data);

// Function to be executed in state drive
// valid return states: NO_CHANGE, STATE_RE_ENABLE_INV_DRIVE, STATE_DRIVE,
// STATE_DISABLE_INV_DRIVE
state_t do_drive(state_data_t *data);

// Function to be executed in state disable_inv_drive
// valid return states: NO_CHANGE, STATE_DISABLE_INV_DRIVE,
// STATE_START_TS_DISCHARGE
state_t do_disable_inv_drive(state_data_t *data);

// Function to be executed in state re_enable_inv_drive
// valid return states: STATE_DRIVE
state_t do_re_enable_inv_drive(state_data_t *data);

// Function to be executed in state wait_ts_discharge
// valid return states: NO_CHANGE, STATE_IDLE, STATE_WAIT_TS_DISCHARGE
state_t do_wait_ts_discharge(state_data_t *data);

// List of state functions
extern state_func_t *const state_table[NUM_STATES];

// No transition functions

// state manager
state_t run_state(state_t cur_state, state_data_t *data);

#endif
