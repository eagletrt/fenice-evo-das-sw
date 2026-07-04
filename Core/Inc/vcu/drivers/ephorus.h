/*!
 * \file ephorus.h
 * \date 2026-06-24
 * \authors Alessandro Bridi [ale.bridi15@gmail.com]
 * \ingroup Core
 *
 * \brief Types, configuration and state for the Ephorus 3.1 inverter driver.
 *
 * \details Pure logic: this driver serializes setpoint frames into raw bytes
 *     and decodes received bytes into telemetry through libcan-sw. It never
 *     touches the CAN bus, the HAL, or any generated peripheral file.
 *
 *     One EphorusHandler drives the whole Ephorus system: it owns up to four
 *     wheels (EphorusWheel, one physical inverter each) plus the telemetry that
 *     the system reports once for everyone (EphorusGeneralTelemetry, decoded
 *     from the shared 0x400/0x401 frames). Per-wheel data lives in the wheel,
 *     shared data lives in \c general - never duplicated.
 */

#ifndef EPHORUS_H
#define EPHORUS_H

#include <stdbool.h>
#include <stdint.h>

#include "can-inverters-api.h"

/*! \brief Payload size of an inverters-network frame (classic CAN, fixed 8). */
#define EPHORUS_FRAME_DATA_SIZE (8U)

/* Shared general frames (carry all four inverters at once). */
#define EPHORUS_RX_GENERAL CAN_INVERTERS_MESSAGE_FRAME_ID_GENERALOUTBOUND /*!< 0x400 DC link / enable mirror */
#define EPHORUS_RX_ERRORS CAN_INVERTERS_MESSAGE_FRAME_ID_GENERALERRORBITS /*!< 0x401 latched faults */

#define EPHORUS_TX_PERIOD_MS (10U)       /*!< Setpoint cadence, hard requirement < 50 ms (inverter comm timeout). */
#define EPHORUS_STATUS_PERIOD_MS (80U)   /*!< Telemetry print cadence. */
#define EPHORUS_DEFAULT_RUN_RPM (100)    /*!< Speed commanded while running. */
#define EPHORUS_DEFAULT_TORQUE_NM (2.0f) /*!< +/- torque window. */
#define EPHORUS_RPM_SLEW (4000.0f)       /*!< Max commanded RPM change per second (ramp). */
#define EPHORUS_MAX_RPM (2500)            /*!< Upper clamp for the run setpoint. */
#define EPHORUS_MAX_TORQUE_NM (60.0f)    /*!< Upper clamp for the torque window. */

/*!
 * \brief The four wheels of the car, each bound to one physical inverter.
 *
 * \details Fixed mapping: front-left = inverter 1 ... rear-right = inverter 4.
 *     Wheels 0-1 share the "12" DC bus, wheels 2-3 the "34" bus.
 */
enum EphorusWheel {
    EPHORUS_WHEEL_FRONT_LEFT = 0,  /*!< inverter 1: setpoints 0x186, outbound 0x383/0x385 */
    EPHORUS_WHEEL_FRONT_RIGHT = 1, /*!< inverter 2: setpoints 0x196, outbound 0x393/0x395 */
    EPHORUS_WHEEL_REAR_LEFT = 2,   /*!< inverter 3: setpoints 0x1A6, outbound 0x3A3/0x3A5 */
    EPHORUS_WHEEL_REAR_RIGHT = 3,  /*!< inverter 4: setpoints 0x1B6, outbound 0x3B3/0x3B5 */
    EPHORUS_WHEEL_COUNT = 4,       /*!< Number of wheels. */
};

/*!
 * \brief Return codes for the Ephorus driver.
 */
enum EphorusReturnCode {
    EPHORUS_RC_OK,              /*!< Operation successful. */
    EPHORUS_RC_NULL_POINTER,    /*!< A null pointer was passed to a function. */
    EPHORUS_RC_INVALID_WHEEL,   /*!< Wheel index out of range. */
    EPHORUS_RC_INACTIVE,        /*!< The wheel has not been attached. */
    EPHORUS_RC_SERIALIZE_ERROR, /*!< libcan-sw failed to serialize the setpoints frame. */
};

/*!
 * \brief Inverter run state, as reported in the InverterState signal.
 */
enum EphorusState {
    EPHORUS_STATE_IDLE = 0,           /*!< Idle. */
    EPHORUS_STATE_DRIVE = 1,          /*!< Drive. */
    EPHORUS_STATE_ERROR = 2,          /*!< Error. */
    EPHORUS_STATE_CONFIG_MISSING = 3, /*!< Config values missing. */
};

/*!
 * \brief Per-wheel latched faults (bit positions in EphorusWheelTelemetry::fault_bits).
 */
enum EphorusWheelFault {
    EPHORUS_WHEEL_FAULT_TIMEOUT_COMM,       /*!< No setpoint received for more than 50 ms. */
    EPHORUS_WHEEL_FAULT_DISABLE_UNDER_LOAD, /*!< Enable went low while motor was under load. */
    EPHORUS_WHEEL_FAULT_POSITION_SENSOR,    /*!< No valid position data. */
    EPHORUS_WHEEL_FAULT_MOTOR_TEMPERATURE,  /*!< Motor temperature measurement faulty. */
    EPHORUS_WHEEL_FAULT_OVERTEMPERATURE,    /*!< Motor or switches overheated. */
    EPHORUS_WHEEL_FAULT_OVERSPEED,          /*!< Speed limit exceeded. */
    EPHORUS_WHEEL_FAULT_CONTROL_ERROR,      /*!< Current controller could not reach setpoint. */
    EPHORUS_WHEEL_FAULT_OVERCURRENT,        /*!< Overcurrent in a phase. */
    EPHORUS_WHEEL_FAULT_SHORT_CIRCUIT,      /*!< Desat protection triggered. */
    EPHORUS_WHEEL_FAULT_SUM_PHASE_CURRENTS, /*!< Sum of phase currents implausible. */
    EPHORUS_WHEEL_FAULT_INTERNAL_FAULT,     /*!< Internal inverter fault. */
    EPHORUS_WHEEL_FAULT_COUNT               /*!< Number of per-wheel faults. */
};

/*!
 * \brief Shared faults (bit positions in EphorusGeneralTelemetry::fault_bits).
 *
 * \details ControlDisabled and LV_Supply are system-wide; the DC faults are per
 *     DC bus (12 = wheels 0-1, 34 = wheels 2-3).
 */
enum EphorusGeneralFault {
    EPHORUS_GENERAL_FAULT_CONTROL_DISABLED,  /*!< Control enable/disable pin glitched while in drive. */
    EPHORUS_GENERAL_FAULT_LV_SUPPLY,         /*!< Low-voltage supply insufficient. */
    EPHORUS_GENERAL_FAULT_DC_UNDERVOLTAGE_12,/*!< DC bus 12 dropped below threshold. */
    EPHORUS_GENERAL_FAULT_DC_OVERVOLTAGE_12, /*!< DC bus 12 exceeded 700 V. */
    EPHORUS_GENERAL_FAULT_DC_UNDERVOLTAGE_34,/*!< DC bus 34 dropped below threshold. */
    EPHORUS_GENERAL_FAULT_DC_OVERVOLTAGE_34, /*!< DC bus 34 exceeded 700 V. */
    EPHORUS_GENERAL_FAULT_COUNT              /*!< Number of shared faults. */
};

/*!
 * \brief Telemetry that is specific to one wheel.
 */
struct EphorusWheelTelemetry {
    enum EphorusState state; /*!< Last reported inverter state. */
    bool ready;              /*!< InverterReady flag. */
    int16_t speed_rpm;       /*!< Actual motor speed [RPM]. */
    float torque_nm;         /*!< Actual motor torque [Nm]. */
    float temp_motor_c;      /*!< Motor temperature [degC]. */
    float temp_switches_c;   /*!< Power switches temperature [degC]. */
    uint32_t fault_bits;     /*!< Bitmask of EphorusWheelFault currently latched. */
};

/*!
 * \brief Telemetry the system reports once for every inverter.
 */
struct EphorusGeneralTelemetry {
    float dclink_voltage_12_v; /*!< DC bus 12 voltage [V] (wheels 0-1). */
    float dclink_voltage_34_v; /*!< DC bus 34 voltage [V] (wheels 2-3). */
    bool dclink_good_12;       /*!< DC bus 12 within range. */
    bool dclink_good_34;       /*!< DC bus 34 within range. */
    bool enable_mirror;        /*!< Mirror of the control-enable input. */
    uint32_t fault_bits;       /*!< Bitmask of EphorusGeneralFault currently latched. */
};

/*!
 * \brief One wheel: command state, frame ids and per-wheel telemetry.
 *
 * Fields are public for rendering, but mutate them only through ephorus_api_*.
 */
struct EphorusWheelState {
    bool active;            /*!< true once attached via ephorus_api_attach. */
    uint32_t tx_id;         /*!< Setpoints frame id. */
    uint32_t outbound_a_id; /*!< OutboundA frame id (state / torque / temps). */
    uint32_t outbound_b_id; /*!< OutboundB frame id (speed). */

    bool armed;       /*!< Inverter enabled (latched errors acked). */
    bool running;     /*!< Run requested (commands the run setpoint). */
    bool faulted;     /*!< A fault was latched, motion inhibited until re-armed. */
    int run_rpm;      /*!< Speed commanded while running [RPM]. */
    float torque_nm;  /*!< +/- torque window [Nm]. */
    float cmd_rpm;    /*!< Slew limited speed actually being sent [RPM]. */
    bool ack_pulse;   /*!< One-shot AckErr rising edge request. */
    bool reset_pulse; /*!< One-shot ResetError request. */

    struct EphorusWheelTelemetry tlm; /*!< Decoded per-wheel telemetry. */
};

/*!
 * \brief Ephorus driver handle: every wheel plus the shared telemetry.
 */
struct EphorusHandler {
    struct EphorusWheelState wheels[EPHORUS_WHEEL_COUNT]; /*!< Per-wheel state. */
    struct EphorusGeneralTelemetry general;          /*!< Shared telemetry (decoded once). */
};

#endif // EPHORUS_H
