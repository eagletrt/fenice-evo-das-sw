#include "vehicle-api.h"
#include "eagletrt.h"
#include "pedals-api.h"
#include <string.h>

EAGLETRT_STATIC struct VehicleHandler vehicle_handler;

enum VehicleReturnCode vehicle_api_init() {
    memset(&vehicle_handler, 0, sizeof(vehicle_handler));
    return VEHICLE_RC_OK;
}

void vehicle_api_request_state(enum VehicleRequestedState state) {
    vehicle_handler.requested_state = state;
}

enum VehicleRequestedState vehicle_api_get_requested_state() {
    return vehicle_handler.requested_state;
}

bool vehicle_api_is_driver_ready() {
    if (pedals_api_is_brake_pressed() && pedals_api_get_requested_throttle_torque() == 0.0f) {
        return true;
    }
    return false;
}

void vehicle_api_set_shutdown_end_closed(bool closed) {
    vehicle_handler.shutdown_end_closed = closed;
}

bool vehicle_api_is_shutdown_end_closed() {
    return vehicle_handler.shutdown_end_closed;
}

bool vehicle_api_is_higher_than_60v() {
    return vehicle_handler.higher_than_60v;
}

void vehicle_api_set_higher_than_60v(bool higher) {
    vehicle_handler.higher_than_60v = higher;
}
