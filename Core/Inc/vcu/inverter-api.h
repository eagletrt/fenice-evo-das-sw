/*!
 * \file inverter-api.h
 * \date 2026-06-26
 * \authors Alessandro Bridi [ale.bridi15@gmail.com]
 * \ingroup Core
 *
 * \brief Public API for the generic Inverter module.
 *
 * Typical usage (single-inverter bench, only the front-right wheel attached):
 * \code
 *   inverter_api_init();
 *   inverter_api_attach(EPHORUS_WHEEL_FRONT_RIGHT);
 *
 *   while (1) {
 *       can_communications_api_process_rx(INVERTER_NETWORK); // -> decode
 *       inverter_api_step(HAL_GetTick());                    // build + queue setpoints
 *       can_communications_api_process_tx(INVERTER_NETWORK); // flush to the bus
 *   }
 * \endcode
 *
 * A four-wheel ECU simply attaches all four wheels.
 */

#ifndef INVERTER_API_H
#define INVERTER_API_H

#include "inverter.h"
#include "can-communications.h"

/*! \brief Reset the module (driver re-initialized, no wheels attached). */
void inverter_api_init(void);

/*!
 * \brief Attach (activate) a wheel so it transmits and decodes.
 *
 * \param wheel Wheel to activate.
 *
 * \retval INVERTER_RC_OK on success.
 * \retval INVERTER_RC_INVALID_WHEEL if \p wheel is out of range.
 */
enum InverterReturnCode inverter_api_attach(enum EphorusWheel wheel);

/*! \brief Arm a wheel (clear latched errors, enable drive). */
void inverter_api_arm(enum EphorusWheel wheel);

/*! \brief Disarm a wheel (stop run request, disable drive). */
void inverter_api_disarm(enum EphorusWheel wheel);

/*! \brief Command or release a wheel's run setpoint. */
void inverter_api_set_run(enum EphorusWheel wheel, bool run);

/*! \brief Flip a wheel's run request. */
void inverter_api_toggle_run(enum EphorusWheel wheel);

/*! \brief Set a wheel's run-speed setpoint [RPM]. */
void inverter_api_set_speed(enum EphorusWheel wheel, int rpm);

/*! \brief Set a wheel's torque window [Nm]. */
void inverter_api_set_torque(enum EphorusWheel wheel, float nm);

/*!
 * \brief Build and queue setpoint frames for every attached wheel when due.
 *
 * \details No-op until EPHORUS_TX_PERIOD_MS has elapsed, so it is safe (and
 *     intended) to call on every main-loop iteration. Frames are pushed into
 *     the can-communications TX queue; call can_communications_api_process_tx
 *     afterwards to flush them.
 *
 * \param tick Current tick.
 *
 * \retval INVERTER_RC_OK frames queued or not yet due.
 * \retval INVERTER_RC_TX_ERROR a TX queue rejected a frame.
 */
enum InverterReturnCode inverter_api_step(uint32_t tick);

/*! \brief Borrow a wheel's telemetry (NULL if \p wheel out of range). */
const struct EphorusWheelTelemetry *inverter_api_wheel_telemetry(enum EphorusWheel wheel);

/*! \brief Borrow the shared telemetry. */
const struct EphorusGeneralTelemetry *inverter_api_general_telemetry(void);

/*! \brief Name of an inverter state (static string, never NULL). */
const char *inverter_api_state_name(enum EphorusState state);

/*! \brief Name of a per-wheel fault bit (static string, never NULL). */
const char *inverter_api_wheel_fault_name(int fault_bit);

/*! \brief Name of a shared fault bit (static string, never NULL). */
const char *inverter_api_general_fault_name(int fault_bit);

/*!
 * \brief Router entry point: decode and dispatch one received frame.
 *
 * \details Matches the can_communications_receive_callback signature. Wire it
 *     up from can_communications_router_api_receive_inverter.
 *
 * \param[in] frame The frame popped off the RX queue.
 *
 * \retval CAN_COMMUNICATION_RC_OK on success.
 * \retval CAN_COMMUNICATION_RC_NULL_POINTER if \p frame is NULL.
 */
enum CanCommunicationReturnCode inverter_api_on_receive(const struct CanCommunicationFrame *frame);

#endif // INVERTER_API_H
