/* Enable CAN-Lib implementation */
#define primary_NETWORK_IMPLEMENTATION
#define secondary_NETWORK_IMPLEMENTATION
#define primary_IDS_IMPLEMENTATION
#define secondary_IDS_IMPLEMENTATION
#define primary_WATCHDOG_IMPLEMENTATION
#define secondary_WATCHDOG_IMPLEMENTATION

#include "can_messages.h"

#include "../Lib/can/lib/inverters/inverters_network.h"
#include "../Lib/can/lib/primary/primary_network.h"
#include "../Lib/can/lib/primary/primary_watchdog.h"
#include "../Lib/can/lib/secondary/secondary_network.h"
#include "../Lib/can/lib/secondary/secondary_watchdog.h"
#include "can.h"
#include "can_fifo_queue.h"
#include "can_user_functions.h"
#include "das_version.h"
#include "inverters.h"
#include "stdlib.h"
#include "string.h"

#ifdef TESTING
#include "../../tests/logger_stubs.h"
#else
#include "logger.h"
#endif

/* Prototypes and inline functions ------------------------------------------ */
bool _CANMSG_needs_to_be_sent(CAN_IdTypeDef, CAN_HandleTypeDef *);
bool _CANMSG_primary_serialize_msg_by_id(CAN_IdTypeDef, CAN_MessageTypeDef *);
bool _CANMSG_secondary_serialize_msg_by_id(CAN_IdTypeDef, CAN_MessageTypeDef *);
void _CANMSG_primary_deserialize_msg_by_id(CAN_MessageTypeDef msg);
void _CANMSG_secondary_deserialize_msg_by_id(CAN_MessageTypeDef msg);

/* Private variables -------------------------------------------------------- */

/* Initialize all static CAN messages with safe contents */

/* Primary Network */
ecumsg_ecu_version_t ecumsg_ecu_version_state = {
    {0U, false, 0U},
    {.component_build_time = INT_COMPONENT_VERSION, .canlib_build_time = CANLIB_BUILD_TIME}};
ecumsg_ecu_errors_t ecumsg_ecu_errors_state                 = {{0U, false, 0U}, {0U}};
ecumsg_ecu_set_power_maps_t ecumsg_ecu_set_power_maps_state = {
    {0U, false, 0U},
    {.map_power = 0.0, .sc_state = 0, .tv_state = 0, .reg_state = 0}};
ecumsg_ecu_power_maps_t ecumsg_ecu_power_maps_state = {{0U, false, 0U}, {.map_power = 0.0, .sc_state = 0, .tv_state = 0, .reg_state = 0}};
ecumsg_ecu_status_t ecumsg_ecu_status_state         = {{0U, false, 0U}, {.status = primary_ecu_status_status_idle}};
ecumsg_ecu_feedbacks_t ecumsg_ecu_feedbacks_state   = {{0U, false, 0U}, {0}};
ecumsg_ecu_set_status_t ecumsg_ecu_set_status_state = {{0U, false, 0U}, {.status = primary_ecu_set_status_status_idle}};
ecumsg_front_angular_velocity_t ecumsg_front_angular_velocity_state = {{0U, false, 0U}, {0}};
// CANMSG_HVVoltageTypeDef             CANMSG_HVVoltage      = { {0U, false, 0U}, { 0U } };
// CANMSG_HVCurrentTypeDef             CANMSG_HVCurrent      = { {0U, false, 0U}, { 0U } };
// CANMSG_HVTemperatureTypeDef         CANMSG_HVTemperature  = { {0U, false, 0U}, { 0U } };
ecumsg_hv_errors_t ecumsg_hv_errors_state                   = {{0U, false, 0U}, {0U}};
ecumsg_hv_feedback_status_t ecumsg_hv_feedback_status_state = {{0U, false, 0U}, {0U}};
ecumsg_hv_status_t ecumsg_hv_status_state                   = {{0U, false, 0U}, {.status = primary_hv_status_status_init}};
ecumsg_hv_set_status_ecu_t ecumsg_hv_set_status_ecu_state   = {
    {0U, false, 0U},
    {.hv_status_set = false}};
// CANMSG_LVCurrentTypeDef             CANMSG_LVCurrent      = { {0U, false, 0U}, { 0U } };
// CANMSG_LVVoltageTypeDef             CANMSG_LVVoltage      = { {0U, false, 0U}, { 0U } };
// CANMSG_LVTemperatureTypeDef         CANMSG_LVTemperature  = { {0U, false, 0U}, { 0U } };
ecumsg_lv_inverter_connection_status_t ecumsg_lv_inverter_connection_status_state = {{0U, false, 0U}, {0U}};
ecumsg_ecu_set_ptt_status_t ecumsg_ecu_set_ptt_status_state                       = {{0U, false, 0U}, {0U}};
ecumsg_ecu_ptt_status_t ecumsg_ecu_ptt_status_state = {{0U, false, 0U}, {.status = primary_ecu_ptt_status_status_off}};
// CANMSG_SetPedalCalibrationTypeDef  CANMSG_SetPedalsCalibration  = { {0U, false, 0U}, { 0U } };
// CANMSG_PedalCalibrationAckTypeDef  CANMSG_PedalsCalibrationAck  = { {0U, false, 0U}, { 0U } };
ecumsg_lv_set_inverter_connection_status_t ecumsg_lv_set_inverter_connection_status_state = {{0U, false, 0U}, {0U}};
ecumsg_tlm_status_t ecumsg_tlm_status_state                                               = {{0U, false, 0U}, {0U}};
ecumsg_hv_total_voltage_t ecumsg_hv_total_voltage_state                                   = {{0U, false, 0U}, {0U}};

// CANMSG_AmbientTemperatureTypeDef CANMSG_AmbientTemperature  = { {0U, false, 0U}, { 0U } };

/* Secondary Network */
// CANMSG_PedValsTypeDef        CANMSG_PedVals        = { {0U, false, 0U}, { 0U } };
ecumsg_control_output_t ecumsg_control_output_state               = {{0U, false, 0U}, {0U}};
ecumsg_steer_angle_t ecumsg_steer_angle_state                     = {{0U, false, 0U}, {0U}};
ecumsg_pedal_throttle_t ecumsg_pedal_throttle_state               = {{0U, false, 0U}, {0U}};
ecumsg_pedal_brakes_pressure_t ecumsg_pedal_brakes_pressure_state = {{0U, false, 0U}, {0U}};
// ecumsg_imu_acceleration_t         CANMSG_IMUAcc         = { {0U, false, 0U}, { 0U } };
// ecumsg_imu_angular_rate_t         CANMSG_IMUAng         = { {0U, false, 0U}, { 0U } };
ecumsg_control_status_t ecumsg_control_status_state                 = {{0U, false, 0U}, {0U}};
ecumsg_ecu_control_status_t ecumsg_ecu_control_status_state         = {{0U, false, 0U}, {0U}};
ecumsg_hv_cells_voltage_stats_t ecumsg_hv_cells_voltage_stats_state = {{0U, false, 0U}, {0U}};
ecumsg_hv_soc_t ecumsg_hv_soc_estimation_state_state                = {{0U, false, 0U}, {0U}};

/*
ecumsg_ecu_set_steer_actuator_angle_t ecumsg_ecu_set_steer_actuator_angle_state = {{0}};
ecumsg_ecu_steer_actuator_status_t ecumsg_ecu_steer_actuator_status_state = {{0}};
ecumsg_ecu_set_steer_actuator_status_tlm_t ecumsg_ecu_set_steer_actuator_status_tlm_state = {{0}};
*/

//ecumsg_as_commands_set_status_t ecumsg_as_commands_set_status_state = {{0}};
//ecumsg_as_commands_status_t ecumsg_as_commands_status_state = {{0}};
//ecumsg_as_commands_set_value_t ecumsg_as_commands_set_value_state = {{0}};

ecumsg_as_commands_status_t ecumsg_as_commands_status_state = {{0U, false, 0U}, {.steerstatus = primary_as_commands_status_steerstatus_off, 
    .brakestatus = primary_as_commands_status_brakestatus_off, .throttlestatus = primary_as_commands_status_throttlestatus_off}};
ecumsg_as_commands_set_status_t ecumsg_as_commands_set_status_state = {{0U, false, 0U}, {.steerstatus = primary_as_commands_status_steerstatus_off, 
    .brakestatus = primary_as_commands_status_brakestatus_off, .throttlestatus = primary_as_commands_status_throttlestatus_off}};
ecumsg_as_commands_set_value_t ecumsg_as_commands_set_value_state = {{0U, false, 0U}, {0U}};


/* Inverter automatic message */
CANMSG_INVResponseTypeDef CANMSG_InvL_I_CMD_RAMP    = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvL_I_CMD         = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvL_IQ_ACTUAL     = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvL_ID_ACTUAL     = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvL_T_MOTOR       = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvL_T_IGBT        = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvL_N_ACTUAL_FILT = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvL_N_CMD_RAMP    = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvL_VDC_BUS       = {{0U, false, 0U}};

CANMSG_INVResponseTypeDef CANMSG_InvR_I_CMD_RAMP    = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvR_I_CMD         = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvR_IQ_ACTUAL     = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvR_ID_ACTUAL     = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvR_T_MOTOR       = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvR_T_IGBT        = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvR_N_ACTUAL_FILT = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvR_N_CMD_RAMP    = {{0U, false, 0U}};
CANMSG_INVResponseTypeDef CANMSG_InvR_VDC_BUS       = {{0U, false, 0U}};

CANFQ_QueueTypeDef _CANMSG_RX_queue = {0U};

void CANMSG_init() {
    CANFQ_init(&_CANMSG_RX_queue);
}

void CANMSG_add_msg_to_RX_queue(CAN_MessageTypeDef *msg) {
    if (!CANFQ_is_full(_CANMSG_RX_queue))
        CANFQ_push(_CANMSG_RX_queue, msg);
}

void CANMSG_process_RX_queue() {
    CAN_MessageTypeDef msg;
    while (CANFQ_pop(_CANMSG_RX_queue, &msg)) {
        if (msg.hcan == &CAN_PRIMARY_NETWORK) {
            if (inverters_id_is_message(msg.id)) {
                INV_parse_CAN_msg(msg.id, msg.data, msg.size);
                // LOG_write(LOGLEVEL_INFO, "INV proc RX queue");
            } else {
                // LOG_write(LOGLEVEL_INFO, "PRI proc RX queue");
                _CANMSG_primary_deserialize_msg_by_id(msg);

                CANMSG_MetadataTypeDef *msg_to_update = CANMSG_get_primary_metadata_from_id(msg.id);
                if (msg_to_update != NULL) {
                    msg_to_update->is_new    = true;
                    msg_to_update->timestamp = HAL_GetTick();
                }
            }
        } else if (msg.hcan == &CAN_SECONDARY_NETWORK) {
            _CANMSG_secondary_deserialize_msg_by_id(msg);
            // LOG_write(LOGLEVEL_INFO, "SEC proc RX queue");

            CANMSG_MetadataTypeDef *msg_to_update = CANMSG_get_secondary_metadata_from_id(msg.id);
            if (msg_to_update != NULL) {
                msg_to_update->timestamp = HAL_GetTick();
                msg_to_update->is_new    = true;
            }
        }
    }
}

void _CANMSG_primary_deserialize_msg_by_id(CAN_MessageTypeDef msg) {
    switch (msg.id) {
        ECU_CANLIB_UNPACK(ecu_set_power_maps, primary, ECU_SET_POWER_MAPS, PRIMARY);
        ECU_CANLIB_UNPACK(ecu_set_status, primary, ECU_SET_STATUS, PRIMARY);
        ECU_CANLIB_UNPACK(hv_status, primary, HV_STATUS, PRIMARY);
        // ECU_CANLIB_UNPACK(hv_voltage, primary, HV_VOLTAGE, PRIMARY);
        // ECU_CANLIB_UNPACK(hv_current, primary, HV_CURRENT, PRIMARY);
        // ECU_CANLIB_UNPACK(hv_temp, primary, HV_TEMP, PRIMARY);
        ECU_CANLIB_UNPACK(hv_errors, primary, HV_ERRORS, PRIMARY);
        ECU_CANLIB_UNPACK(hv_feedback_status, primary, HV_FEEDBACK_STATUS, PRIMARY);
        // ECU_CANLIB_UNPACK(pedal calibration);
        ECU_CANLIB_UNPACK(lv_inverter_connection_status, primary, LV_INVERTER_CONNECTION_STATUS, PRIMARY);
        ECU_CANLIB_UNPACK(ecu_set_ptt_status, primary, ECU_SET_PTT_STATUS, PRIMARY);
        ECU_CANLIB_UNPACK(control_output, primary, CONTROL_OUTPUT, PRIMARY);
        ECU_CANLIB_UNPACK(control_status, primary, CONTROL_STATUS, PRIMARY);
        ECU_CANLIB_UNPACK(tlm_status, primary, TLM_STATUS, PRIMARY);
        ECU_CANLIB_UNPACK(hv_total_voltage, primary, HV_TOTAL_VOLTAGE, PRIMARY);
        ECU_CANLIB_UNPACK(hv_cells_voltage_stats, primary, HV_CELLS_VOLTAGE_STATS, PRIMARY);

        //ECU_CANLIB_UNPACK(ecu_set_steer_actuator_status_tlm, primary, ECU_SET_STEER_ACTUATOR_STATUS_TLM, PRIMARY);
        //ECU_CANLIB_UNPACK(ecu_set_steer_actuator_angle, primary, ECU_SET_STEER_ACTUATOR_ANGLE, PRIMARY);
        ECU_CANLIB_UNPACK(as_commands_set_status, primary, AS_COMMANDS_SET_STATUS, PRIMARY);
        ECU_CANLIB_UNPACK(as_commands_set_value, primary, AS_COMMANDS_SET_VALUE, PRIMARY);
        default:
            // LOG_write(LOGLEVEL_ERR, "[CANMSG/Deserialize] Unknown message id: 0x%X", msg.id);
            break;
    }
}

void _CANMSG_secondary_deserialize_msg_by_id(CAN_MessageTypeDef msg) {
    switch (msg.id) {
        ECU_CANLIB_UNPACK(hv_soc_estimation_state, secondary, HV_SOC_ESTIMATION_STATE, SECONDARY);
        default:
            // LOG_write(LOGLEVEL_ERR, "[CANMSG/Deserialize] Unknown message id: 0x%X", msg.id);
            break;
    }
}

/**
 * @brief     Return a pointer to the metadata struct for the specified CAN message of primary network
 */
CANMSG_MetadataTypeDef *CANMSG_get_primary_metadata_from_id(CAN_IdTypeDef id) {
    switch (id) {
        case PRIMARY_ECU_VERSION_FRAME_ID:
            return &(ecumsg_ecu_version_state.info);
        case PRIMARY_ECU_ERRORS_FRAME_ID:
            return &(ecumsg_ecu_errors_state.info);
        case PRIMARY_ECU_SET_POWER_MAPS_FRAME_ID:
            return &(ecumsg_ecu_set_power_maps_state.info);
        case PRIMARY_ECU_POWER_MAPS_FRAME_ID:
            return &(ecumsg_ecu_power_maps_state.info);
        case PRIMARY_ECU_STATUS_FRAME_ID:
            return &(ecumsg_ecu_status_state.info);
        case PRIMARY_ECU_FEEDBACKS_FRAME_ID:
            return &(ecumsg_ecu_feedbacks_state.info);
        case PRIMARY_ECU_SET_STATUS_FRAME_ID:
            return &(ecumsg_ecu_set_status_state.info);
        case PRIMARY_ECU_CONTROL_STATUS_FRAME_ID:
            return &(ecumsg_ecu_control_status_state.info);
        case PRIMARY_HV_SET_STATUS_ECU_FRAME_ID:
            return &(ecumsg_hv_set_status_ecu_state.info);
        case PRIMARY_HV_STATUS_FRAME_ID:
            return &(ecumsg_hv_status_state.info);
        case PRIMARY_HV_TOTAL_VOLTAGE_FRAME_ID:
            return &(ecumsg_hv_total_voltage_state.info);
        case PRIMARY_HV_CELLS_VOLTAGE_STATS_FRAME_ID:
            return &(ecumsg_hv_cells_voltage_stats_state.info);
        case PRIMARY_HV_ERRORS_FRAME_ID:
            return &(ecumsg_hv_errors_state.info);
        case PRIMARY_HV_FEEDBACK_STATUS_FRAME_ID:
            return &(ecumsg_hv_feedback_status_state.info);
        case PRIMARY_LV_INVERTER_CONNECTION_STATUS_FRAME_ID:
            return &(ecumsg_lv_inverter_connection_status_state.info);
        case PRIMARY_ECU_SET_PTT_STATUS_FRAME_ID:
            return &(ecumsg_ecu_set_ptt_status_state.info);
        case PRIMARY_ECU_PTT_STATUS_FRAME_ID:
            return &(ecumsg_ecu_ptt_status_state.info);
        case PRIMARY_LV_SET_INVERTER_CONNECTION_STATUS_FRAME_ID:
            return &(ecumsg_lv_set_inverter_connection_status_state.info);
        // case PRIMARY_AMBIENT_TEMPERATURE_FRAME_ID:
        // return &(CANMSG_AmbientTemperature.info);
        case PRIMARY_CONTROL_OUTPUT_FRAME_ID:
            return &(ecumsg_control_output_state.info);
        case PRIMARY_CONTROL_STATUS_FRAME_ID:
            return &(ecumsg_control_status_state.info);
        case PRIMARY_TLM_STATUS_FRAME_ID:
            return &(ecumsg_tlm_status_state.info);
        //case PRIMARY_ECU_SET_STEER_ACTUATOR_STATUS_TLM_FRAME_ID:
        //    return &(ecumsg_ecu_set_steer_actuator_status_tlm_state.info);
        //case PRIMARY_ECU_SET_STEER_ACTUATOR_ANGLE_FRAME_ID:
        //    return &(ecumsg_ecu_set_steer_actuator_angle_state.info);
        //case PRIMARY_ECU_STEER_ACTUATOR_STATUS_FRAME_ID:
        //    return &(ecumsg_ecu_steer_actuator_status_state.info);
        case PRIMARY_AS_COMMANDS_STATUS_FRAME_ID:
            return &(ecumsg_as_commands_status_state.info);
        case PRIMARY_AS_COMMANDS_SET_STATUS_FRAME_ID:
            return &(ecumsg_as_commands_set_status_state.info);
        case PRIMARY_AS_COMMANDS_SET_VALUE_FRAME_ID:
            return &(ecumsg_as_commands_set_value_state.info);
        // case PRIMARY_SET_CELL_BALANCING_STATUS_FRAME_ID:
        // case PRIMARY_HANDCART_SETTINGS_SET_FRAME_ID:
        // case PRIMARY_TLM_VERSION_FRAME_ID:
        // return NULL;
        default:
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata] Unknown message id: 0x%X", id);

            // uint8_t* name[30] = { 0U };
            // primary_message_name_from_id(id, name);
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata]     > primary nwk decoding: [%s]", name);
            //
            // name[0] = '\0';
            // inverters_message_name_from_id(id, name);
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata]     > inverters nwk decoding: [%s]", name);
            return NULL;
    }
}

/**
 * @brief     Return a pointer to the metadata struct for the specified CAN message of secondary network
 */
CANMSG_MetadataTypeDef *CANMSG_get_secondary_metadata_from_id(CAN_IdTypeDef id) {
    switch (id) {
        case SECONDARY_FRONT_ANGULAR_VELOCITY_FRAME_ID:
            return &(ecumsg_front_angular_velocity_state.info);
        case SECONDARY_PEDAL_BRAKES_PRESSURE_FRAME_ID:
            return &(ecumsg_pedal_brakes_pressure_state.info);
        case SECONDARY_PEDAL_THROTTLE_FRAME_ID:
            return &(ecumsg_pedal_throttle_state.info);
        // case SECONDARY_PEDALS_OUTPUT_FRAME_ID:
        // return &(CANMSG_PedVals.info);
        case SECONDARY_STEER_ANGLE_FRAME_ID:
            return &(ecumsg_steer_angle_state.info);
        case SECONDARY_HV_SOC_ESTIMATION_STATE_FRAME_ID:
            return &(ecumsg_hv_soc_estimation_state_state.info);
        // case SECONDARY_IMU_ACCELERATION_FRAME_ID:
        // return &(CANMSG_IMUAcc.info);
        // case SECONDARY_IMU_ANGULAR_RATE_FRAME_ID:
        // return &(CANMSG_IMUAng.info);
        default:
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata] Unknown message id: 0x%X", id);
            // name[0] = '\0';
            // secondary_message_name_from_id(id, name);
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata]     > secondary nwk decoding: [%s]", name);
            return NULL;
    }
}

/**
 * @brief     Return a pointer to the metadata struct for the specified CAN message of inverter messages
 */
CANMSG_MetadataTypeDef *CANMSG_get_InvL_metadata_from_mux_id(CAN_IdTypeDef id) {
    switch (id) {
        case INVERTERS_INV_L_RCV_RCV_MUX_ID_22_I_CMD_RAMP_CHOICE:
            return &(CANMSG_InvL_I_CMD_RAMP.info);
        case INVERTERS_INV_L_RCV_RCV_MUX_ID_26_I_CMD_CHOICE:
            return &(CANMSG_InvL_I_CMD.info);
        case INVERTERS_INV_L_RCV_RCV_MUX_ID_27_IQ_ACTUAL_CHOICE:
            return &(CANMSG_InvL_IQ_ACTUAL.info);
        case INVERTERS_INV_L_RCV_RCV_MUX_ID_49_T_MOTOR_CHOICE:
            return &(CANMSG_InvL_T_MOTOR.info);
        case INVERTERS_INV_L_RCV_RCV_MUX_ID_4A_T_IGBT_CHOICE:
            return &(CANMSG_InvL_T_IGBT.info);
        case INVERTERS_INV_L_RCV_RCV_MUX_ID_A8_N_ACTUAL_FILT_CHOICE:
            return &(CANMSG_InvL_N_ACTUAL_FILT.info);
        case INVERTERS_INV_L_RCV_RCV_MUX_ID_32_N_CMD_RAMP_CHOICE:
            return &(CANMSG_InvL_N_CMD_RAMP.info);
        case INVERTERS_INV_L_SEND_READ_ID_EBH_VDC_BUS_CHOICE:
            return &(CANMSG_InvL_VDC_BUS.info);
        default:
            return NULL;
    }
}
/**
 * @brief     Return a pointer to the metadata struct for the specified CAN message of inverter messages
 */
CANMSG_MetadataTypeDef *CANMSG_get_InvR_metadata_from_mux_id(CAN_IdTypeDef id) {
    switch (id) {
        case INVERTERS_INV_R_RCV_RCV_MUX_ID_22_I_CMD_RAMP_CHOICE:
            return &(CANMSG_InvR_I_CMD_RAMP.info);
        case INVERTERS_INV_R_RCV_RCV_MUX_ID_26_I_CMD_CHOICE:
            return &(CANMSG_InvR_I_CMD.info);
        case INVERTERS_INV_R_RCV_RCV_MUX_ID_27_IQ_ACTUAL_CHOICE:
            return &(CANMSG_InvR_IQ_ACTUAL.info);
        case INVERTERS_INV_R_RCV_RCV_MUX_ID_49_T_MOTOR_CHOICE:
            return &(CANMSG_InvR_T_MOTOR.info);
        case INVERTERS_INV_R_RCV_RCV_MUX_ID_4A_T_IGBT_CHOICE:
            return &(CANMSG_InvR_T_IGBT.info);
        case INVERTERS_INV_R_RCV_RCV_MUX_ID_A8_N_ACTUAL_FILT_CHOICE:
            return &(CANMSG_InvR_N_ACTUAL_FILT.info);
        case INVERTERS_INV_R_RCV_RCV_MUX_ID_32_N_CMD_RAMP_CHOICE:
            return &(CANMSG_InvR_N_CMD_RAMP.info);
        case INVERTERS_INV_R_SEND_READ_ID_EBH_VDC_BUS_CHOICE:
            return &(CANMSG_InvR_VDC_BUS.info);
        default:
            return NULL;
    }
}

/**
 * @brief Loop through all the messages to be sent. If mailboxes are full, 
 *        stop and wait for the next call. The index is static, so the loop
 *        will begin from the last position.
 */
void CANMSG_flush_TX() {
    /* Static set of IDs that need to be sent */
    static CAN_IdTypeDef primary_ids_to_send[] = {
        PRIMARY_ECU_VERSION_FRAME_ID,
        PRIMARY_ECU_ERRORS_FRAME_ID,
        PRIMARY_ECU_STATUS_FRAME_ID,
        PRIMARY_ECU_FEEDBACKS_FRAME_ID,
        PRIMARY_ECU_CONTROL_STATUS_FRAME_ID,
        PRIMARY_HV_SET_STATUS_ECU_FRAME_ID,
        PRIMARY_LV_SET_INVERTER_CONNECTION_STATUS_FRAME_ID,
        PRIMARY_ECU_POWER_MAPS_FRAME_ID,
        // PRIMARY_AMBIENT_TEMPERATURE_FRAME_ID,
        PRIMARY_ECU_PTT_STATUS_FRAME_ID,
        PRIMARY_AS_COMMANDS_STATUS_FRAME_ID,
    };
    static CAN_IdTypeDef secondary_ids_to_send[] = {
        SECONDARY_FRONT_ANGULAR_VELOCITY_FRAME_ID,
        SECONDARY_PEDAL_BRAKES_PRESSURE_FRAME_ID,
        SECONDARY_PEDAL_THROTTLE_FRAME_ID,
        SECONDARY_STEER_ANGLE_FRAME_ID,
    };
    // static uint8_t ids_loop_idx = 0;
    uint8_t primary_ids_len   = sizeof(primary_ids_to_send) / sizeof(primary_ids_to_send[0]);
    uint8_t secondary_ids_len = sizeof(secondary_ids_to_send) / sizeof(secondary_ids_to_send[0]);
    uint8_t ids_loop_len      = primary_ids_len + secondary_ids_len;

    /* Loop until either MBs are full or everything has been sent */
    for (uint16_t tx_count = 0; tx_count < ids_loop_len; tx_count++) {
        CAN_IdTypeDef id;
        CANMSG_MetadataTypeDef *info;

        if (tx_count < primary_ids_len) {
            id   = primary_ids_to_send[tx_count];
            info = CANMSG_get_primary_metadata_from_id(id);
            if (info == NULL) {
                continue;
            }
            info->hcan = &CAN_PRIMARY_NETWORK;
        } else {
            id   = secondary_ids_to_send[tx_count - primary_ids_len];
            info = CANMSG_get_secondary_metadata_from_id(id);
            if (info == NULL) {
                continue;
            }
            info->hcan = &CAN_SECONDARY_NETWORK;
        }

        /* Send if interval is elapsed or interval is "once" and msg is new */
        if (_CANMSG_needs_to_be_sent(id, info->hcan)) {
            CAN_MessageTypeDef msg;

            if (info->hcan == &CAN_PRIMARY_NETWORK) {
                if (_CANMSG_primary_serialize_msg_by_id(id, &msg)) {
                    if (CAN_send(&msg, info->hcan) != HAL_OK) {
                        LOG_write(LOGLEVEL_ERR, "CAN SEND ERROR PRIMARY: 0x%X", id);
                    }
                } else {
                    LOG_write(LOGLEVEL_WARN, "[CANMSG/flushTX] Failed to serialize message 0x%X from primary network", id);
                }

            } else {
                if (_CANMSG_secondary_serialize_msg_by_id(id, &msg)) {
                    if (CAN_send(&msg, info->hcan) != HAL_OK) {
                        LOG_write(LOGLEVEL_ERR, "CAN SEND ERROR SECONDARY: 0x%X", id);
                    }
                } else {
                    LOG_write(LOGLEVEL_WARN, "[CANMSG/flushTX] Failed to serialize message 0x%X from secondary network", id);
                }
            }
            info->timestamp = HAL_GetTick();
            info->is_new    = false;
        }
    }
}

/**
 * @brief Check if a message needs to be sent. That is, if it is new and its interval
 *        is "once", or has elapsed since the last transmission.
 */
bool _CANMSG_needs_to_be_sent(CAN_IdTypeDef id, CAN_HandleTypeDef *nwk) {
    CANMSG_MetadataTypeDef *info;
    if (nwk == &CAN_PRIMARY_NETWORK) {
        info = CANMSG_get_primary_metadata_from_id(id);
    } else {
        info = CANMSG_get_secondary_metadata_from_id(id);
    }
    if (info == NULL) {
        return false;
    }
    int32_t interval = (nwk == &CAN_PRIMARY_NETWORK) ? primary_watchdog_interval_from_id(id) : secondary_watchdog_interval_from_id(id);
    int32_t elapsed  = HAL_GetTick() - info->timestamp;
    if (!info->is_new) {
        return false;
    }
    if (interval == -1 && elapsed > 50) {
        return true;
    } else {
        return (elapsed >= interval && interval != -1);
    }
}

bool _CANMSG_primary_serialize_msg_by_id(CAN_IdTypeDef id, CAN_MessageTypeDef *msg) {
    msg->id = id;

    switch (id) {
        ECU_CANLIB_PACK(ecu_version, primary, ECU_VERSION, PRIMARY);
        ECU_CANLIB_PACK(ecu_errors, primary, ECU_ERRORS, PRIMARY);
        ECU_CANLIB_PACK(ecu_status, primary, ECU_STATUS, PRIMARY);
        ECU_CANLIB_PACK(ecu_feedbacks, primary, ECU_FEEDBACKS, PRIMARY);
        ECU_CANLIB_PACK(ecu_control_status, primary, ECU_CONTROL_STATUS, PRIMARY);
        ECU_CANLIB_PACK(hv_set_status_ecu, primary, HV_SET_STATUS_ECU, PRIMARY);
        ECU_CANLIB_PACK(ecu_power_maps, primary, ECU_POWER_MAPS, PRIMARY);
        ECU_CANLIB_PACK(lv_set_inverter_connection_status, primary, LV_SET_INVERTER_CONNECTION_STATUS, PRIMARY);
        ECU_CANLIB_PACK(ecu_ptt_status, primary, ECU_PTT_STATUS, PRIMARY);
        ECU_CANLIB_PACK(control_output, primary, CONTROL_OUTPUT, PRIMARY);
        ECU_CANLIB_PACK(as_commands_status, primary, AS_COMMANDS_STATUS, PRIMARY);
        default:
            LOG_write(LOGLEVEL_ERR, "[CANMSG/Serialize] Unknown message id: 0x%X", msg->id);

            char name[30] = {0U};
            primary_message_name_from_id(id, name);
            LOG_write(LOGLEVEL_WARN, "[CANMSG/Serialize]     > primary nwk decoding: [%s]", name);

            name[0] = '\0';
            secondary_message_name_from_id(id, name);
            LOG_write(LOGLEVEL_WARN, "[CANMSG/Serialize]     > secondary nwk decoding: [%s]", name);

            name[0] = '\0';
            inverters_message_name_from_id(id, name);
            LOG_write(LOGLEVEL_WARN, "[CANMSG/Serialize]     > inverters nwk decoding: [%s]", name);
            return false;
    }

    return true;
}

bool _CANMSG_secondary_serialize_msg_by_id(CAN_IdTypeDef id, CAN_MessageTypeDef *msg) {
    msg->id = id;

    switch (id) {
        ECU_CANLIB_PACK(front_angular_velocity, secondary, FRONT_ANGULAR_VELOCITY, SECONDARY);
        ECU_CANLIB_PACK(pedal_throttle, secondary, PEDAL_THROTTLE, SECONDARY);
        ECU_CANLIB_PACK(pedal_brakes_pressure, secondary, PEDAL_BRAKES_PRESSURE, SECONDARY);
        ECU_CANLIB_PACK(steer_angle, secondary, STEER_ANGLE, SECONDARY);
        default:
            LOG_write(LOGLEVEL_ERR, "[CANMSG/Serialize] Unknown message id: 0x%X", msg->id);

            char name[30] = {0U};
            primary_message_name_from_id(id, name);
            LOG_write(LOGLEVEL_WARN, "[CANMSG/Serialize]     > primary nwk decoding: [%s]", name);

            name[0] = '\0';
            secondary_message_name_from_id(id, name);
            LOG_write(LOGLEVEL_WARN, "[CANMSG/Serialize]     > secondary nwk decoding: [%s]", name);

            name[0] = '\0';
            inverters_message_name_from_id(id, name);
            LOG_write(LOGLEVEL_WARN, "[CANMSG/Serialize]     > inverters nwk decoding: [%s]", name);
            return false;
    }

    return true;
}
