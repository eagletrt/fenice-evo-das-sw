#ifndef VEHICLE_API_H
#define VEHICLE_API_H

#include "vehicle.h"
#include "fsm.h"

enum VehicleReturnCode vehicle_api_init(void);

void vehicle_api_request_state(enum VehicleRequestedState state);

enum VehicleRequestedState vehicle_api_get_requested_state(void);

bool vehicle_api_is_driver_ready(void);

void vehicle_api_set_shutdown_end_closed(bool closed);

bool vehicle_api_is_shutdown_end_closed(void);

bool vehicle_api_is_higher_than_60v(void);

void vehicle_api_set_higher_than_60v(bool higher);

enum VehicleReturnCode vehicle_api_periodically_send_state(fsm_state_t state, uint32_t tick);

enum VehicleReturnCode vehicle_api_periodically_send_identity(uint32_t tick);

#endif // VEHICLE_API_H
