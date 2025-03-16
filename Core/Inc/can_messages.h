#ifndef CANMSG_H
#define CANMSG_H

#include "../Lib/can/lib/primary/primary_network.h"
#include "../Lib/can/lib/secondary/secondary_network.h"
#include "can.h"
#include "stdbool.h"

typedef struct {
    uint32_t timestamp;      /*< Timestamp of when the message was received/sent */
    bool is_new;             /*< If the message data has yet to be processed     */
    CAN_HandleTypeDef *hcan; /*< CAN peripheral used to send/receive the message */
} CANMSG_MetadataTypeDef;

#define ECU_CANLIB_UNPACK(msg_name, ntw, MSG_NAME, NTW)                                                         \
    case NTW##_##MSG_NAME##_FRAME_ID: {                                                                         \
        ntw##_##msg_name##_t raw_##msg_name##_data;                                                             \
        ntw##_##msg_name##_unpack(&raw_##msg_name##_data, msg.data, NTW##_##MSG_NAME##_BYTE_SIZE);              \
        ntw##_##msg_name##_raw_to_conversion_struct(&(ecumsg_##msg_name##_state.data), &raw_##msg_name##_data); \
        ecumsg_##msg_name##_state.info.hcan = msg.hcan;                                                         \
        break;                                                                                                  \
    }

#define ECU_CANLIB_PACK(msg_name, ntw, MSG_NAME, NTW)                                                           \
    case NTW##_##MSG_NAME##_FRAME_ID: {                                                                         \
        ntw##_##msg_name##_t raw_##msg_name##_data;                                                             \
        ntw##_##msg_name##_conversion_to_raw_struct(&raw_##msg_name##_data, &(ecumsg_##msg_name##_state.data)); \
        msg->size = ntw##_##msg_name##_pack(msg->data, &raw_##msg_name##_data, NTW##_##MSG_NAME##_BYTE_SIZE);   \
        break;                                                                                                  \
    }

/* Primary Network */
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_version_converted_t data;
} ecumsg_ecu_version_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_errors_converted_t data;
} ecumsg_ecu_errors_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_set_power_maps_converted_t data;
} ecumsg_ecu_set_power_maps_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_power_maps_converted_t data;
} ecumsg_ecu_power_maps_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_status_converted_t data;
} ecumsg_ecu_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_feedbacks_converted_t data;
} ecumsg_ecu_feedbacks_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_set_status_converted_t data;
} ecumsg_ecu_set_status_t;
// typedef struct { CANMSG_MetadataTypeDef info; primary_hv_total_voltage_converted_t data;       } CANMSG_HVVoltageTypeDef;
// typedef struct { CANMSG_MetadataTypeDef info; primary_hv_current_t data;       } CANMSG_HVCurrentTypeDef;
// typedef struct { CANMSG_MetadataTypeDef info; primary_hv_cells_temp_converted_t data;          } CANMSG_HVTemperatureTypeDef;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_hv_errors_converted_t data;
} ecumsg_hv_errors_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_hv_feedback_status_converted_t data;
} ecumsg_hv_feedback_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_hv_status_converted_t data;
} ecumsg_hv_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_hv_set_status_ecu_converted_t data;
} ecumsg_hv_set_status_ecu_t;
// typedef struct { CANMSG_MetadataTypeDef info; primary_lv_currents_converted_t data;       } CANMSG_LVCurrentTypeDef;
// typedef struct { CANMSG_MetadataTypeDef info; primary_lv_cells_voltage_t data;       } CANMSG_LVVoltageTypeDef;
// typedef struct { CANMSG_MetadataTypeDef info; primary_lv_cells_temp_t data;   } CANMSG_LVTemperatureTypeDef;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_lv_inverter_connection_status_converted_t data;
} ecumsg_lv_inverter_connection_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_set_ptt_status_converted_t data;
} ecumsg_ecu_set_ptt_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_ptt_status_converted_t data;
} ecumsg_ecu_ptt_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_lv_set_inverter_connection_status_converted_t data;
} ecumsg_lv_set_inverter_connection_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_tlm_status_converted_t data;
} ecumsg_tlm_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_hv_total_voltage_converted_t data;
} ecumsg_hv_total_voltage_t;

typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_hv_cells_voltage_stats_converted_t data;
} ecumsg_hv_cells_voltage_stats_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    secondary_hv_soc_estimation_state_converted_t data;
} ecumsg_hv_soc_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_control_output_converted_t data;
} ecumsg_control_output_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_control_status_converted_t data;
} ecumsg_control_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_control_status_converted_t data;
} ecumsg_ecu_control_status_t;

/* Secondary Network */
typedef struct {
    CANMSG_MetadataTypeDef info;
    secondary_front_angular_velocity_converted_t data;
} ecumsg_front_angular_velocity_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    secondary_pedal_throttle_converted_t data;
} ecumsg_pedal_throttle_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    secondary_pedal_brakes_pressure_converted_t data;
} ecumsg_pedal_brakes_pressure_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    secondary_steer_angle_converted_t data;
} ecumsg_steer_angle_t;

/*
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_set_steer_actuator_angle_converted_t data;
} ecumsg_ecu_set_steer_actuator_angle_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_steer_actuator_status_converted_t data;
} ecumsg_ecu_steer_actuator_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_ecu_set_steer_actuator_status_tlm_converted_t data;
} ecumsg_ecu_set_steer_actuator_status_tlm_t;
*/

typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_as_commands_status_t data;
} ecumsg_as_commands_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_as_commands_set_status_t data;
} ecumsg_as_commands_set_status_t;
typedef struct {
    CANMSG_MetadataTypeDef info;
    primary_as_commands_set_value_t data;
} ecumsg_as_commands_set_value_t;

/* Inverter automatic message */
typedef struct {
    CANMSG_MetadataTypeDef info;
} CANMSG_INVResponseTypeDef;

extern ecumsg_ecu_version_t ecumsg_ecu_version_state;
extern ecumsg_ecu_errors_t ecumsg_ecu_errors_state;
extern ecumsg_ecu_set_power_maps_t ecumsg_ecu_set_power_maps_state;
extern ecumsg_ecu_power_maps_t ecumsg_ecu_power_maps_state;
extern ecumsg_ecu_status_t ecumsg_ecu_status_state;
extern ecumsg_ecu_feedbacks_t ecumsg_ecu_feedbacks_state;
extern ecumsg_ecu_set_status_t ecumsg_ecu_set_status_state;
extern ecumsg_ecu_control_status_t ecumsg_ecu_control_status_state;
extern ecumsg_front_angular_velocity_t ecumsg_front_angular_velocity_state;
extern ecumsg_hv_errors_t ecumsg_hv_errors_state;
extern ecumsg_hv_feedback_status_t ecumsg_hv_feedback_status_state;
extern ecumsg_hv_status_t ecumsg_hv_status_state;
extern ecumsg_hv_set_status_ecu_t ecumsg_hv_set_status_ecu_state;
extern ecumsg_lv_inverter_connection_status_t ecumsg_lv_inverter_connection_status_state;
extern ecumsg_lv_set_inverter_connection_status_t ecumsg_lv_set_inverter_connection_status_state;
extern ecumsg_ecu_set_ptt_status_t ecumsg_ecu_set_ptt_status_state;
extern ecumsg_ecu_ptt_status_t ecumsg_ecu_ptt_status_state;
extern ecumsg_tlm_status_t ecumsg_tlm_status_state;
extern ecumsg_hv_total_voltage_t ecumsg_hv_total_voltage_state;
extern ecumsg_hv_cells_voltage_stats_t ecumsg_hv_cells_voltage_stats_state;

extern ecumsg_pedal_throttle_t ecumsg_pedal_throttle_state;
extern ecumsg_pedal_brakes_pressure_t ecumsg_pedal_brakes_pressure_state;
extern ecumsg_steer_angle_t ecumsg_steer_angle_state;

/*
extern ecumsg_ecu_set_steer_actuator_angle_t ecumsg_ecu_set_steer_actuator_angle_state;
extern ecumsg_ecu_steer_actuator_status_t ecumsg_ecu_steer_actuator_status_state;
extern ecumsg_ecu_set_steer_actuator_status_tlm_t ecumsg_ecu_set_steer_actuator_status_tlm_state;
*/

extern ecumsg_as_commands_status_t ecumsg_as_commands_status_state;
extern ecumsg_as_commands_set_status_t ecumsg_as_commands_set_status_state;
extern ecumsg_as_commands_set_value_t ecumsg_as_commands_set_value_state;

extern ecumsg_hv_soc_t ecumsg_hv_soc_estimation_state_state;
extern ecumsg_control_output_t ecumsg_control_output_state;
extern ecumsg_control_status_t ecumsg_control_status_state;

extern CANMSG_INVResponseTypeDef CANMSG_InvL_I_CMD_RAMP, CANMSG_InvL_I_CMD, CANMSG_InvL_IQ_ACTUAL, CANMSG_InvL_ID_ACTUAL,
    CANMSG_InvL_T_MOTOR, CANMSG_InvL_T_IGBT, CANMSG_InvL_N_ACTUAL_FILT, CANMSG_InvL_N_CMD_RAMP, CANMSG_InvL_VDC_BUS;
extern CANMSG_INVResponseTypeDef CANMSG_InvR_I_CMD_RAMP, CANMSG_InvR_I_CMD, CANMSG_InvR_IQ_ACTUAL, CANMSG_InvR_ID_ACTUAL,
    CANMSG_InvR_T_MOTOR, CANMSG_InvR_T_IGBT, CANMSG_InvR_N_ACTUAL_FILT, CANMSG_InvR_N_CMD_RAMP, CANMSG_InvR_VDC_BUS;

/**
 * Implementation of CAN messages
 * ==============================
 * 
 * When a new message arrives, call CANMSG_add_msg_to_RX_queue() which will add it to a SW FIFO queue
 * without slowing down the IRQ. Then, in the main loop periodically call CANMSG_process_RX_queue() which
 * will empty this queue and deserialize each message into the corresponding struct.
 * 
 * This approach allows each module to access the messages in which it is interested as global variables
 * without having to read the whole queue.
 * 
*/

/**
 * @brief Initialize all queues
 */
void CANMSG_init();

/**
 * @brief Extract a new message from the peripheral and store it in the buffer
 */
void CANMSG_add_msg_to_RX_queue(CAN_MessageTypeDef *msg);

/**
 * @brief Send all new/available messages
 */
void CANMSG_flush_TX();

/**
 * @brief Read all new/available messages in the RX buffer
 */
void CANMSG_process_RX_queue();

CANMSG_MetadataTypeDef *CANMSG_get_primary_metadata_from_id(CAN_IdTypeDef id);
CANMSG_MetadataTypeDef *CANMSG_get_secondary_metadata_from_id(CAN_IdTypeDef id);
CANMSG_MetadataTypeDef *CANMSG_get_InvL_metadata_from_mux_id(CAN_IdTypeDef id);
CANMSG_MetadataTypeDef *CANMSG_get_InvR_metadata_from_mux_id(CAN_IdTypeDef id);

#endif