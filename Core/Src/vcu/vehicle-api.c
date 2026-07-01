#define _XOPEN_SOURCE
#include "vehicle-api.h"

#include "can-communications-api.h"
#include "can-primary-api.h"
#include "eagletrt.h"
#include "pedals-api.h"
#include <time.h>

#include <string.h>

EAGLETRT_STATIC struct VehicleHandler vehicle_handler;

enum VehicleReturnCode vehicle_api_init() {
    memset(&vehicle_handler, 0, sizeof(vehicle_handler));
    struct tm timeinfo;
    strptime(__DATE__ " " __TIME__, "%b %d %Y %H:%M:%S", &timeinfo);
    vehicle_handler.build_time = mktime(&timeinfo);
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

enum VehicleReturnCode vehicle_api_periodically_send_state(fsm_state_t state, uint32_t tick) {
    EAGLETRT_STATIC uint32_t last_send_tick = 0;
    if (tick - last_send_tick >= 200) {
        last_send_tick = tick;
        union CanPrimaryMessages message = { 0 };
        struct CanCommunicationFrame frame = { 0 };
        message.ecu_status.name = state;
        frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_ECU_STATUS;
        frame.length = can_primary_byte_size_ecu_status;
        if (can_primary_api_serialize_from_id(frame.id, &message, frame.data) == -1) {
            return VEHICLE_RC_ERROR;
        }
        if (can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame) != CAN_COMMUNICATION_RC_OK) {
            return VEHICLE_RC_ERROR;
        }
    }
    return VEHICLE_RC_OK;
}

enum VehicleReturnCode vehicle_api_periodically_send_identity(uint32_t tick) {
    EAGLETRT_STATIC uint32_t last_send_tick = 0;
    if (tick - last_send_tick >= 1000) {
        last_send_tick = tick;
        union CanPrimaryMessages message = { 0 };
        struct CanCommunicationFrame frame = { 0 };
        message.ecu_version.canlibbuildtime_s = can_generation_time;
        message.ecu_version.buildtime_s = vehicle_handler.build_time;
        frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_ECU_VERSION;
        frame.length = can_primary_byte_size_ecu_version;
        if (can_primary_api_serialize_from_id(frame.id, &message, frame.data) == -1) {
            return VEHICLE_RC_ERROR;
        }
        if (can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame) != CAN_COMMUNICATION_RC_OK) {
            return VEHICLE_RC_ERROR;
        }
    }
    return VEHICLE_RC_OK;
}
