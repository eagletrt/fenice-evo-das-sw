/*!
 * \file ephorus-api.h
 * \date 2026-06-24
 * \authors Alessandro Bridi [ale.bridi15@gmail.com]
 * \ingroup Core
 *
 * \brief Ephorus 3.1 inverter driver API.
 *
 * \details Pure logic. It serializes setpoint frames into raw bytes and decodes
 *     received bytes into telemetry using libcan-sw - it never touches the CAN
 *     bus, the HAL, or any generated peripheral file. The CAN transport and
 *     timing are owned by the generic Inverter module, which holds an
 *     EphorusHandler by value and calls these functions directly.
 */

#ifndef EPHORUS_API_H
#define EPHORUS_API_H

#include "ephorus.h"

/*!
 * \brief Reset the driver: configure every wheel's frame ids, none active.
 *
 * \param handle Driver handle to initialize.
 *
 * \retval EPHORUS_RC_OK Initialization successful.
 * \retval EPHORUS_RC_NULL_POINTER handle was NULL.
 */
enum EphorusReturnCode ephorus_api_init(struct EphorusHandler *handle);

/*!
 * \brief Activate a wheel so it starts transmitting and decoding.
 *
 * \param handle Driver handle.
 * \param wheel  Wheel to activate.
 *
 * \retval EPHORUS_RC_OK on success.
 * \retval EPHORUS_RC_NULL_POINTER handle was NULL.
 * \retval EPHORUS_RC_INVALID_WHEEL wheel out of range.
 */
enum EphorusReturnCode ephorus_api_attach(struct EphorusHandler *handle, enum EphorusWheel wheel);

/*! \brief Arm a wheel (clear latched errors, enable drive). */
void ephorus_api_arm(struct EphorusHandler *handle, enum EphorusWheel wheel);

/*! \brief Disarm a wheel (stop run request, disable drive). */
void ephorus_api_disarm(struct EphorusHandler *handle, enum EphorusWheel wheel);

/*! \brief Command or release a wheel's run setpoint (only while armed/unfaulted). */
void ephorus_api_set_run(struct EphorusHandler *handle, enum EphorusWheel wheel, bool run);

/*! \brief Flip a wheel's run request. */
void ephorus_api_toggle_run(struct EphorusHandler *handle, enum EphorusWheel wheel);

/*! \brief Set a wheel's run speed [RPM] (clamped, slew-limited). */
void ephorus_api_set_speed(struct EphorusHandler *handle, enum EphorusWheel wheel, int rpm);

/*! \brief Set a wheel's symmetric torque window [Nm] (clamped). */
void ephorus_api_set_torque(struct EphorusHandler *handle, enum EphorusWheel wheel, float nm);

/*!
 * \brief Applies one slew step and serializes a wheel's setpoint frame.
 *
 * \details Meant to be called once per EPHORUS_TX_PERIOD_MS (the Inverter
 *     module enforces that cadence).
 *
 * \param[in]  handle Driver handle.
 * \param[in]  wheel  Wheel to serialize.
 * \param[out] id     CAN identifier to send under.
 * \param[out] data   Payload buffer (EPHORUS_FRAME_DATA_SIZE bytes).
 *
 * \retval EPHORUS_RC_OK Frame serialized.
 * \retval EPHORUS_RC_NULL_POINTER a pointer argument was NULL.
 * \retval EPHORUS_RC_INVALID_WHEEL wheel out of range.
 * \retval EPHORUS_RC_INACTIVE the wheel is not attached (nothing to send).
 * \retval EPHORUS_RC_SERIALIZE_ERROR libcan-sw failed to encode the frame.
 */
enum EphorusReturnCode ephorus_api_build_setpoints(struct EphorusHandler *handle, enum EphorusWheel wheel, uint32_t *id, uint8_t data[EPHORUS_FRAME_DATA_SIZE]);

/*!
 * \brief Decodes one received frame and routes it to the right wheel / general.
 *
 * \details Wheel-specific outbound frames update that wheel; the shared
 *     0x400/0x401 frames update the general telemetry and every active wheel's
 *     faults. Frames that are not part of the inverters network are ignored.
 *     When a new fault is latched the affected wheels' run request is cleared.
 *
 * \param handle Driver handle.
 * \param id     CAN identifier of the received frame.
 * \param data   Payload bytes (EPHORUS_FRAME_DATA_SIZE bytes).
 */
void ephorus_api_handle_frame(struct EphorusHandler *handle, uint32_t id, const uint8_t data[EPHORUS_FRAME_DATA_SIZE]);

/*!
 * \brief Borrow a wheel's telemetry (valid while the handle lives).
 *
 * \param handle Driver handle.
 * \param wheel  Wheel to read.
 * \return Pointer into the handle, or NULL if \p wheel is out of range.
 */
const struct EphorusWheelTelemetry *ephorus_api_wheel_telemetry(const struct EphorusHandler *handle, enum EphorusWheel wheel);

/*!
 * \brief Borrow the shared telemetry (valid while the handle lives).
 *
 * \param handle Driver handle.
 * \return Pointer into the handle, or NULL if \p handle is NULL.
 */
const struct EphorusGeneralTelemetry *ephorus_api_general_telemetry(const struct EphorusHandler *handle);

/*! \brief Name of an inverter state (static string, never NULL). */
const char *ephorus_api_state_name(enum EphorusState state);

/*! \brief Name of a per-wheel fault bit (static string, never NULL). */
const char *ephorus_api_wheel_fault_name(int fault_bit);

/*! \brief Name of a shared fault bit (static string, never NULL). */
const char *ephorus_api_general_fault_name(int fault_bit);

#endif // EPHORUS_API_H
