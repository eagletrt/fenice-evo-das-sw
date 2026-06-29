/*!
 * \file ephorus-api.c
 * \date 2026-06-24
 * \authors Alessandro Bridi [ale.bridi15@gmail.com]
 * \ingroup Core
 *
 * \brief Implementation of the Ephorus 3.1 inverter driver.
 *
 * Pure logic: serialize/deserialize only. No HAL, no CAN transport.
 */

#include "ephorus-api.h"

#include "can-inverters-api.h"
#include "eagletrt-api.h"

/*!
 * \brief Per-wheel fault mask for inverter \c N (token-pasted field names).
 *
 * \details Defined once and stamped out per inverter so the fault list lives in
 *     a single place: (2) -> err_inverter2_overcurrent, ...
 */
#define EPHORUS_WHEEL_FAULT_MASK(e, N)                                                                   \
    (((e)->err_inverter##N##_timeout_comm ? (1U << EPHORUS_WHEEL_FAULT_TIMEOUT_COMM) : 0U) |             \
     ((e)->err_inverter##N##_disable_under_load ? (1U << EPHORUS_WHEEL_FAULT_DISABLE_UNDER_LOAD) : 0U) | \
     ((e)->err_inverter##N##_position_sensor ? (1U << EPHORUS_WHEEL_FAULT_POSITION_SENSOR) : 0U) |       \
     ((e)->err_inverter##N##_motortemperature ? (1U << EPHORUS_WHEEL_FAULT_MOTOR_TEMPERATURE) : 0U) |    \
     ((e)->err_inverter##N##_overtemperature ? (1U << EPHORUS_WHEEL_FAULT_OVERTEMPERATURE) : 0U) |       \
     ((e)->err_inverter##N##_overspeed ? (1U << EPHORUS_WHEEL_FAULT_OVERSPEED) : 0U) |                   \
     ((e)->err_inverter##N##_controlerror ? (1U << EPHORUS_WHEEL_FAULT_CONTROL_ERROR) : 0U) |            \
     ((e)->err_inverter##N##_overcurrent ? (1U << EPHORUS_WHEEL_FAULT_OVERCURRENT) : 0U) |               \
     ((e)->err_inverter##N##_short_circuit ? (1U << EPHORUS_WHEEL_FAULT_SHORT_CIRCUIT) : 0U) |           \
     ((e)->err_inverter##N##_sum_phase_currents ? (1U << EPHORUS_WHEEL_FAULT_SUM_PHASE_CURRENTS) : 0U) | \
     ((e)->err_inverter##N##_internal_fault ? (1U << EPHORUS_WHEEL_FAULT_INTERNAL_FAULT) : 0U))

/*! \brief true if the wheel sits on the "12" DC bus (front), else the "34" bus. */
static bool ephorus_wheel_is_pair_12(enum EphorusWheel wheel) {
    return wheel <= EPHORUS_WHEEL_FRONT_RIGHT;
}

/*! \brief Selects the setpoints / outbound frame ids for a wheel. */
static void ephorus_wheel_frame_ids(enum EphorusWheel wheel, uint32_t *tx_id, uint32_t *outbound_a_id, uint32_t *outbound_b_id) {
    switch (wheel) {
        case EPHORUS_WHEEL_FRONT_LEFT:
            *tx_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER1SETPOINTS;
            *outbound_a_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER1OUTBOUNDA;
            *outbound_b_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER1OUTBOUNDB;
            break;
        case EPHORUS_WHEEL_FRONT_RIGHT:
            *tx_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER2SETPOINTS;
            *outbound_a_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER2OUTBOUNDA;
            *outbound_b_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER2OUTBOUNDB;
            break;
        case EPHORUS_WHEEL_REAR_LEFT:
            *tx_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER3SETPOINTS;
            *outbound_a_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER3OUTBOUNDA;
            *outbound_b_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER3OUTBOUNDB;
            break;
        case EPHORUS_WHEEL_REAR_RIGHT:
        default:
            *tx_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER4SETPOINTS;
            *outbound_a_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER4OUTBOUNDA;
            *outbound_b_id = CAN_INVERTERS_MESSAGE_FRAME_ID_INVERTER4OUTBOUNDB;
            break;
    }
}

/*! \brief Returns the wheel if it exists and is attached, else NULL. */
static struct EphorusWheelState *ephorus_active_wheel(struct EphorusHandler *handle, enum EphorusWheel wheel) {
    if (handle == NULL || wheel >= EPHORUS_WHEEL_COUNT) {
        return NULL;
    }
    struct EphorusWheelState *w = &handle->wheels[wheel];
    return w->active ? w : NULL;
}

/*! \brief Maps a raw InverterState signal to the EphorusState enum. */
static enum EphorusState ephorus_state_from_raw(uint8_t raw) {
    switch (raw) {
        case EPHORUS_STATE_IDLE:
        case EPHORUS_STATE_DRIVE:
        case EPHORUS_STATE_ERROR:
        case EPHORUS_STATE_CONFIG_MISSING:
            return (enum EphorusState)raw;
        default:
            return EPHORUS_STATE_ERROR;
    }
}

/*! \brief Extracts this wheel's per-inverter faults from a decoded error frame. */
static uint32_t ephorus_wheel_fault_bits(const struct CanInvertersGeneralerrorbits *e, enum EphorusWheel wheel) {
    switch (wheel) {
        case EPHORUS_WHEEL_FRONT_LEFT:
            return EPHORUS_WHEEL_FAULT_MASK(e, 1);
        case EPHORUS_WHEEL_FRONT_RIGHT:
            return EPHORUS_WHEEL_FAULT_MASK(e, 2);
        case EPHORUS_WHEEL_REAR_LEFT:
            return EPHORUS_WHEEL_FAULT_MASK(e, 3);
        case EPHORUS_WHEEL_REAR_RIGHT:
        default:
            return EPHORUS_WHEEL_FAULT_MASK(e, 4);
    }
}

/*! \brief Extracts the shared faults from a decoded error frame. */
static uint32_t ephorus_general_fault_bits(const struct CanInvertersGeneralerrorbits *e) {
    uint32_t bits = 0;
    if (e->err_controldisabled)
        bits |= 1U << EPHORUS_GENERAL_FAULT_CONTROL_DISABLED;
    if (e->err_lv_supply)
        bits |= 1U << EPHORUS_GENERAL_FAULT_LV_SUPPLY;
    if (e->err_inverter1_2_dc_undervoltage)
        bits |= 1U << EPHORUS_GENERAL_FAULT_DC_UNDERVOLTAGE_12;
    if (e->err_inverter1_2_dc_overvoltage)
        bits |= 1U << EPHORUS_GENERAL_FAULT_DC_OVERVOLTAGE_12;
    if (e->err_inverter3_4_dc_undervoltage)
        bits |= 1U << EPHORUS_GENERAL_FAULT_DC_UNDERVOLTAGE_34;
    if (e->err_inverter3_4_dc_overvoltage)
        bits |= 1U << EPHORUS_GENERAL_FAULT_DC_OVERVOLTAGE_34;
    return bits;
}

/*! \brief true if any shared fault (global or this wheel's DC bus) is set. */
static bool ephorus_general_faults_hit_wheel(uint32_t general_bits, enum EphorusWheel wheel) {
    uint32_t global = (1U << EPHORUS_GENERAL_FAULT_CONTROL_DISABLED) | (1U << EPHORUS_GENERAL_FAULT_LV_SUPPLY);
    uint32_t dc = ephorus_wheel_is_pair_12(wheel)
                      ? ((1U << EPHORUS_GENERAL_FAULT_DC_UNDERVOLTAGE_12) | (1U << EPHORUS_GENERAL_FAULT_DC_OVERVOLTAGE_12))
                      : ((1U << EPHORUS_GENERAL_FAULT_DC_UNDERVOLTAGE_34) | (1U << EPHORUS_GENERAL_FAULT_DC_OVERVOLTAGE_34));
    return (general_bits & (global | dc)) != 0;
}

/* Per-frame appliers. Outbound A/B and the setpoint structs share one layout
 * across all four inverters, so the canonical inverter1* union member always
 * reads the bytes deserialize_from_id() wrote, whichever inverter it is for. */

static void ephorus_apply_outbound_a(struct EphorusWheelState *w, const struct CanInvertersInverter1outbounda *a) {
    w->tlm.state = ephorus_state_from_raw(a->inverterstate);
    w->tlm.ready = a->inverterready != 0;
    w->tlm.torque_nm = a->torqueactual;
    w->tlm.temp_motor_c = a->temperaturemotor;
    w->tlm.temp_switches_c = a->temperaturepowerswitches;
}

static void ephorus_apply_outbound_b(struct EphorusWheelState *w, const struct CanInvertersInverter1outboundb *b) {
    w->tlm.speed_rpm = b->speedactual;
}

static void ephorus_apply_general(struct EphorusHandler *handle, const struct CanInvertersGeneraloutbound *g) {
    handle->general.dclink_voltage_12_v = g->dclinkvoltage12actual;
    handle->general.dclink_voltage_34_v = g->dclinkvoltage34actual;
    handle->general.dclink_good_12 = g->dclinkgood12 != 0;
    handle->general.dclink_good_34 = g->dclinkgood34 != 0;
    handle->general.enable_mirror = g->mirrorcontrolenable != 0;
}

static void ephorus_apply_errors(struct EphorusHandler *handle, const struct CanInvertersGeneralerrorbits *e) {
    uint32_t general_bits = ephorus_general_fault_bits(e);
    handle->general.fault_bits = general_bits;

    for (enum EphorusWheel wheel = 0; wheel < EPHORUS_WHEEL_COUNT; wheel++) {
        struct EphorusWheelState *w = &handle->wheels[wheel];
        if (!w->active) {
            continue;
        }
        w->tlm.fault_bits = ephorus_wheel_fault_bits(e, wheel);
        bool faulted = w->tlm.fault_bits != 0 || ephorus_general_faults_hit_wheel(general_bits, wheel);
        /* Latch on the rising edge: a fresh fault inhibits motion until re-armed. */
        if (faulted && !w->faulted) {
            w->faulted = true;
            w->running = false;
        }
    }
}

enum EphorusReturnCode ephorus_api_init(struct EphorusHandler *handle) {
    if (handle == NULL) {
        return EPHORUS_RC_NULL_POINTER;
    }

    *handle = (struct EphorusHandler){ 0 };
    for (enum EphorusWheel wheel = 0; wheel < EPHORUS_WHEEL_COUNT; wheel++) {
        struct EphorusWheelState *w = &handle->wheels[wheel];
        ephorus_wheel_frame_ids(wheel, &w->tx_id, &w->outbound_a_id, &w->outbound_b_id);
        w->run_rpm = EPHORUS_DEFAULT_RUN_RPM;
        w->torque_nm = EPHORUS_DEFAULT_TORQUE_NM;
    }
    return EPHORUS_RC_OK;
}

enum EphorusReturnCode ephorus_api_attach(struct EphorusHandler *handle, enum EphorusWheel wheel) {
    if (handle == NULL) {
        return EPHORUS_RC_NULL_POINTER;
    }
    if (wheel >= EPHORUS_WHEEL_COUNT) {
        return EPHORUS_RC_INVALID_WHEEL;
    }
    handle->wheels[wheel].active = true;
    return EPHORUS_RC_OK;
}

void ephorus_api_arm(struct EphorusHandler *handle, enum EphorusWheel wheel) {
    struct EphorusWheelState *w = ephorus_active_wheel(handle, wheel);
    if (w == NULL) {
        return;
    }
    w->faulted = false;
    w->ack_pulse = true;   /* rising edge AckErr while still disabled */
    w->reset_pulse = true; /* request a latched-error reset */
    w->armed = true;
}

void ephorus_api_disarm(struct EphorusHandler *handle, enum EphorusWheel wheel) {
    struct EphorusWheelState *w = ephorus_active_wheel(handle, wheel);
    if (w == NULL) {
        return;
    }
    w->running = false;
    w->armed = false;
}

void ephorus_api_set_run(struct EphorusHandler *handle, enum EphorusWheel wheel, bool run) {
    struct EphorusWheelState *w = ephorus_active_wheel(handle, wheel);
    if (w != NULL) {
        w->running = run;
    }
}

void ephorus_api_toggle_run(struct EphorusHandler *handle, enum EphorusWheel wheel) {
    struct EphorusWheelState *w = ephorus_active_wheel(handle, wheel);
    if (w != NULL) {
        w->running = !w->running;
    }
}

void ephorus_api_set_speed(struct EphorusHandler *handle, enum EphorusWheel wheel, int rpm) {
    struct EphorusWheelState *w = ephorus_active_wheel(handle, wheel);
    if (w != NULL) {
        w->run_rpm = (int)EAGLETRT_API_CLAMP(rpm, 0, EPHORUS_MAX_RPM);
    }
}

void ephorus_api_set_torque(struct EphorusHandler *handle, enum EphorusWheel wheel, float nm) {
    struct EphorusWheelState *w = ephorus_active_wheel(handle, wheel);
    if (w != NULL) {
        w->torque_nm = EAGLETRT_API_CLAMP(nm, 0.0f, EPHORUS_MAX_TORQUE_NM);
    }
}

enum EphorusReturnCode ephorus_api_build_setpoints(struct EphorusHandler *handle, enum EphorusWheel wheel, uint32_t *id, uint8_t data[EPHORUS_FRAME_DATA_SIZE]) {
    if (handle == NULL || id == NULL || data == NULL) {
        return EPHORUS_RC_NULL_POINTER;
    }
    if (wheel >= EPHORUS_WHEEL_COUNT) {
        return EPHORUS_RC_INVALID_WHEEL;
    }
    struct EphorusWheelState *w = &handle->wheels[wheel];
    if (!w->active) {
        return EPHORUS_RC_INACTIVE;
    }

    const bool drive = w->armed && !w->faulted;
    const float target = (w->running && drive) ? (float)w->run_rpm : 0.0f;

    /* Slew limit one TX period worth of change toward the target speed. */
    const float step = EPHORUS_RPM_SLEW * ((float)EPHORUS_TX_PERIOD_MS / 1000.0f);
    if (w->cmd_rpm < target) {
        w->cmd_rpm = EAGLETRT_API_MIN(target, w->cmd_rpm + step);
    } else if (w->cmd_rpm > target) {
        w->cmd_rpm = EAGLETRT_API_MAX(target, w->cmd_rpm - step);
    }

    /* Consume the one-shot pulses. */
    const bool ack = w->ack_pulse;
    const bool reset = w->reset_pulse;
    w->ack_pulse = false;
    w->reset_pulse = false;

    /* All inverterNsetpoints share one layout in the union; fill the canonical
     * member and let serialize_from_id() encode it under this wheel's tx id. */
    union CanInvertersMessages msg = { 0 };
    msg.inverter1setpoints = (struct CanInvertersInverter1setpoints){
        .enableinverter = drive ? 1 : 0,
        .reseterror = reset ? 1 : 0,
        .ascallowed = 0,
        .currentcontrol = 0, /* never current mode over this network */
        .ackerr = ack ? 1 : 0,
        .speedsetpoint = (int16_t)EAGLETRT_API_MAX(0.0f, w->cmd_rpm),
        .torquelimitpositive = w->torque_nm,
        .torquelimitnegative = -w->torque_nm, /* allow braking torque for a controlled stop */
    };

    if (can_inverters_api_serialize_from_id((enum CanInvertersMessageFrameId)w->tx_id, &msg, data) < 0) {
        return EPHORUS_RC_SERIALIZE_ERROR;
    }

    *id = w->tx_id;
    return EPHORUS_RC_OK;
}

void ephorus_api_handle_frame(struct EphorusHandler *handle, uint32_t id, const uint8_t data[EPHORUS_FRAME_DATA_SIZE]) {
    if (handle == NULL || data == NULL) {
        return;
    }
    if (!can_inverters_api_id_is_valid((enum CanInvertersMessageFrameId)id)) {
        return;
    }

    union CanInvertersMessages msg = { 0 };
    if (can_inverters_api_deserialize_from_id((enum CanInvertersMessageFrameId)id, (uint8_t *)data, &msg) != 0) {
        return;
    }

    if (id == EPHORUS_RX_GENERAL) {
        ephorus_apply_general(handle, &msg.generaloutbound);
        return;
    }
    if (id == EPHORUS_RX_ERRORS) {
        ephorus_apply_errors(handle, &msg.generalerrorbits);
        return;
    }

    /* Wheel-specific outbound frame: route to the matching active wheel. */
    for (enum EphorusWheel wheel = 0; wheel < EPHORUS_WHEEL_COUNT; wheel++) {
        struct EphorusWheelState *w = &handle->wheels[wheel];
        if (!w->active) {
            continue;
        }
        if (id == w->outbound_a_id) {
            ephorus_apply_outbound_a(w, &msg.inverter1outbounda);
            return;
        }
        if (id == w->outbound_b_id) {
            ephorus_apply_outbound_b(w, &msg.inverter1outboundb);
            return;
        }
    }
}

const struct EphorusWheelTelemetry *ephorus_api_wheel_telemetry(const struct EphorusHandler *handle, enum EphorusWheel wheel) {
    if (handle == NULL || wheel >= EPHORUS_WHEEL_COUNT) {
        return NULL;
    }
    return &handle->wheels[wheel].tlm;
}

const struct EphorusGeneralTelemetry *ephorus_api_general_telemetry(const struct EphorusHandler *handle) {
    return handle == NULL ? NULL : &handle->general;
}

const char *ephorus_api_state_name(enum EphorusState state) {
    switch (state) {
        case EPHORUS_STATE_IDLE:
            return "Idle";
        case EPHORUS_STATE_DRIVE:
            return "Drive";
        case EPHORUS_STATE_ERROR:
            return "ERROR";
        case EPHORUS_STATE_CONFIG_MISSING:
            return "CONFIG MISSING";
        default:
            return "?";
    }
}

const char *ephorus_api_wheel_fault_name(int fault_bit) {
    switch ((enum EphorusWheelFault)fault_bit) {
        case EPHORUS_WHEEL_FAULT_TIMEOUT_COMM:
            return "Timeout_Comm";
        case EPHORUS_WHEEL_FAULT_DISABLE_UNDER_LOAD:
            return "Disable_Under_Load";
        case EPHORUS_WHEEL_FAULT_POSITION_SENSOR:
            return "Position_Sensor";
        case EPHORUS_WHEEL_FAULT_MOTOR_TEMPERATURE:
            return "Motortemperature";
        case EPHORUS_WHEEL_FAULT_OVERTEMPERATURE:
            return "Overtemperature";
        case EPHORUS_WHEEL_FAULT_OVERSPEED:
            return "Overspeed";
        case EPHORUS_WHEEL_FAULT_CONTROL_ERROR:
            return "ControlError";
        case EPHORUS_WHEEL_FAULT_OVERCURRENT:
            return "Overcurrent";
        case EPHORUS_WHEEL_FAULT_SHORT_CIRCUIT:
            return "Short_Circuit";
        case EPHORUS_WHEEL_FAULT_SUM_PHASE_CURRENTS:
            return "Sum_Phase_Currents";
        case EPHORUS_WHEEL_FAULT_INTERNAL_FAULT:
            return "Internal_Fault";
        default:
            return "?";
    }
}

const char *ephorus_api_general_fault_name(int fault_bit) {
    switch ((enum EphorusGeneralFault)fault_bit) {
        case EPHORUS_GENERAL_FAULT_CONTROL_DISABLED:
            return "ControlDisabled";
        case EPHORUS_GENERAL_FAULT_LV_SUPPLY:
            return "LV_Supply";
        case EPHORUS_GENERAL_FAULT_DC_UNDERVOLTAGE_12:
            return "DC12_Undervoltage";
        case EPHORUS_GENERAL_FAULT_DC_OVERVOLTAGE_12:
            return "DC12_Overvoltage";
        case EPHORUS_GENERAL_FAULT_DC_UNDERVOLTAGE_34:
            return "DC34_Undervoltage";
        case EPHORUS_GENERAL_FAULT_DC_OVERVOLTAGE_34:
            return "DC34_Overvoltage";
        default:
            return "?";
    }
}
