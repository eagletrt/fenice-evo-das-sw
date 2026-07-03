#include "eagletrt.h"
#include "pedals-api.h"
#include "string.h"

/*!
 * \brief Internal module handler.
 * \details Hidden from external linkage to enforce API-only access.
 */
EAGLETRT_STATIC struct PedalsHandler pedals_handler;

enum PedalsReturnCode pedals_api_init(pedals_set_brake_light_callback set_brake_light, pedals_get_tick_callback get_tick) {
    if (set_brake_light == NULL || pedals_handler.set_brake_light != NULL) {
        return PEDALS_RC_ERROR;
    }

    memset(&pedals_handler, 0U, sizeof(pedals_handler));
    pedals_handler.set_brake_light = set_brake_light;
    pedals_handler.get_tick = get_tick;
    return PEDALS_RC_OK;
}

enum PedalsReturnCode pedals_api_set_throttle(float throttle) {
    if (throttle < 0.0f || throttle > 1.0f) {
        return PEDALS_RC_ERROR;
    }

    pedals_handler.throttle = throttle;
    pedals_handler.last_tick = (pedals_handler.get_tick != NULL) ? pedals_handler.get_tick() : 0U;
    return PEDALS_RC_OK;
}

bool pedals_api_communication_timeout(void) {
    if (pedals_handler.get_tick == NULL) {
        return true;
    }

    uint32_t current_tick = pedals_handler.get_tick();
    return (current_tick - pedals_handler.last_tick) > PEDALS_COMMUNICATION_TIMEOUT_MS;
}

enum PedalsReturnCode pedals_api_set_brake(float brake) {
    if (brake < 0.0f || brake > 1.0f) {
        return PEDALS_RC_ERROR;
    }

    pedals_handler.brake = brake;
    return PEDALS_RC_OK;
}

enum PedalsReturnCode pedals_api_set_brake_pressure(float brake_pressure) {
    if (brake_pressure < 0.0f || brake_pressure > PEDALS_MAX_BRAKE_PRESSURE) {
        return PEDALS_RC_ERROR;
    }

    pedals_handler.brake_pressure = brake_pressure;
    if (pedals_handler.set_brake_light != NULL) {
        pedals_handler.set_brake_light(pedals_api_is_brake_pressed());
    }

    return PEDALS_RC_OK;
}

float pedals_api_get_requested_throttle_torque() {
    return PEDALS_MAX_TORQUE_NM * pedals_handler.throttle;
}

bool pedals_api_is_brake_pressed() {
    return (pedals_handler.brake_pressure >= PEDALS_BRAKE_THRESHOLD_LIGHT_BAR);
}

float pedals_api_get_throttle() {
    return pedals_handler.throttle;
}

float pedals_api_get_brake() {
    return pedals_handler.brake;
}

float pedals_api_get_brake_pressure() {
    return pedals_handler.brake_pressure;
}

void pedals_api_set_brake_light(bool on) {
    if (pedals_handler.set_brake_light != NULL) {
        pedals_handler.set_brake_light(on);
    }
}
