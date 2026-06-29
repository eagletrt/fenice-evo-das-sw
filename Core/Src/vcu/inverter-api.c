/*!
 * \file inverter-api.c
 * \date 2026-06-26
 * \authors Alessandro Bridi [ale.bridi15@gmail.com]
 * \ingroup Core
 *
 * \brief Implementation of the generic Inverter module.
 *
 * \details Holds the concrete Ephorus driver by value and calls ephorus_api_*
 *     directly. To use a different inverter, swap the EphorusHandler member in
 *     struct InverterHandler and the ephorus_api_* calls below for the new
 *     driver's API - the public inverter_api_* surface stays identical.
 */

#include "inverter-api.h"

#include "ephorus-api.h"
#include "can-communications-api.h"

/* The driver fills a fixed-size payload; it must match the transport's frame. */
static_assert(EPHORUS_FRAME_DATA_SIZE == CAN_COMMUNICATIONS_FRAME_DATA_SIZE, "inverter frame payload size mismatch");

/*! \brief File-static module state (one per board). */
static struct InverterHandler inverter_handler;

void inverter_api_init(void) {
    ephorus_api_init(&inverter_handler.driver);
    inverter_handler.last_tx_tick = 0;
}

enum InverterReturnCode inverter_api_attach(enum EphorusWheel wheel) {
    if (ephorus_api_attach(&inverter_handler.driver, wheel) != EPHORUS_RC_OK) {
        return INVERTER_RC_INVALID_WHEEL;
    }
    return INVERTER_RC_OK;
}

void inverter_api_arm(enum EphorusWheel wheel) {
    ephorus_api_arm(&inverter_handler.driver, wheel);
}

void inverter_api_disarm(enum EphorusWheel wheel) {
    ephorus_api_disarm(&inverter_handler.driver, wheel);
}

void inverter_api_set_run(enum EphorusWheel wheel, bool run) {
    ephorus_api_set_run(&inverter_handler.driver, wheel, run);
}

void inverter_api_toggle_run(enum EphorusWheel wheel) {
    ephorus_api_toggle_run(&inverter_handler.driver, wheel);
}

void inverter_api_set_speed(enum EphorusWheel wheel, int rpm) {
    ephorus_api_set_speed(&inverter_handler.driver, wheel, rpm);
}

void inverter_api_set_torque(enum EphorusWheel wheel, float nm) {
    ephorus_api_set_torque(&inverter_handler.driver, wheel, nm);
}

enum InverterReturnCode inverter_api_step(uint32_t tick) {
    if ((tick - inverter_handler.last_tx_tick) < EPHORUS_TX_PERIOD_MS) {
        return INVERTER_RC_OK;
    }
    inverter_handler.last_tx_tick = tick;

    enum InverterReturnCode rc = INVERTER_RC_OK;
    for (enum EphorusWheel wheel = 0; wheel < EPHORUS_WHEEL_COUNT; wheel++) {
        struct CanCommunicationFrame frame = { 0 };
        uint32_t id = 0;
        enum EphorusReturnCode build = ephorus_api_build_setpoints(&inverter_handler.driver, wheel, &id, frame.data);
        if (build == EPHORUS_RC_INACTIVE) {
            continue; /* wheel not attached */
        }
        if (build != EPHORUS_RC_OK) {
            rc = INVERTER_RC_TX_ERROR;
            continue;
        }
        frame.id = id;
        frame.length = EPHORUS_FRAME_DATA_SIZE;
        if (can_communications_api_add_to_tx_buffer(INVERTER_NETWORK, &frame) != CAN_COMMUNICATION_RC_OK) {
            rc = INVERTER_RC_TX_ERROR;
        }
    }
    return rc;
}

const struct EphorusWheelTelemetry *inverter_api_wheel_telemetry(enum EphorusWheel wheel) {
    return ephorus_api_wheel_telemetry(&inverter_handler.driver, wheel);
}

const struct EphorusGeneralTelemetry *inverter_api_general_telemetry(void) {
    return ephorus_api_general_telemetry(&inverter_handler.driver);
}

const char *inverter_api_state_name(enum EphorusState state) {
    return ephorus_api_state_name(state);
}

const char *inverter_api_wheel_fault_name(int fault_bit) {
    return ephorus_api_wheel_fault_name(fault_bit);
}

const char *inverter_api_general_fault_name(int fault_bit) {
    return ephorus_api_general_fault_name(fault_bit);
}

enum CanCommunicationReturnCode inverter_api_on_receive(const struct CanCommunicationFrame *frame) {
    if (frame == NULL) {
        return CAN_COMMUNICATION_RC_NULL_POINTER;
    }
    ephorus_api_handle_frame(&inverter_handler.driver, frame->id, frame->data);
    return CAN_COMMUNICATION_RC_OK;
}
