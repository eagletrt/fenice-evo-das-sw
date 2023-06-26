/* Enable CAN-Lib implementation */
#define primary_NETWORK_IMPLEMENTATION
#define secondary_NETWORK_IMPLEMENTATION
#define primary_IDS_IMPLEMENTATION
#define secondary_IDS_IMPLEMENTATION
#define primary_WATCHDOG_IMPLEMENTATION
#define secondary_WATCHDOG_IMPLEMENTATION

#include "can_messages.h"

// #include "das_version.h"
#include "can.h"
#include "can_user_functions.h"
#include "stdlib.h"
#include "inverters.h"
#include "string.h"
#include "can_fifo_queue.h"

#include "../Lib/can/lib/primary/primary_network.h"
#include "../Lib/can/lib/primary/primary_watchdog.h"
#include "../Lib/can/lib/secondary/secondary_network.h"
#include "../Lib/can/lib/secondary/secondary_watchdog.h"
#include "../Lib/can/lib/inverters/inverters_network.h"

#ifdef TESTING
    #include "../../tests/logger_stubs.h"
#else
    #include "logger.h"
#endif


/* Prototypes and inline functions ------------------------------------------ */
CAN_HandleTypeDef* _CANMSG_get_nwk_from_id(CAN_IdTypeDef);
void _CANMSG_deserialize_msg_by_id(CAN_MessageTypeDef);
bool _CANMSG_needs_to_be_sent(CAN_IdTypeDef, CAN_HandleTypeDef*);
bool _CANMSG_serialize_msg_by_id(CAN_IdTypeDef, CAN_MessageTypeDef*);


/* Private variables -------------------------------------------------------- */

/* Initialize all static CAN messages with safe contents */

/* Primary Network */
CANMSG_DASVersionTypeDef         CANMSG_DASVersion     = { {0U, false}, { 0U } };
CANMSG_DASErrorsTypeDef          CANMSG_DASErrors      = { {0U, false}, { 0U } };
CANMSG_SteerStatusTypeDef        CANMSG_SteerStatus    = { {0U, false}, { .map_pw = 0.5, .map_sc = 0, .map_tv = 0 } };
CANMSG_CarStatusTypeDef          CANMSG_CarStatus      = { {0U, false}, { .car_status = primary_car_status_car_status_IDLE } };
CANMSG_SetCarStatusTypeDef       CANMSG_SetCarStatus   = { {0U, false}, { .car_status_set = primary_set_car_status_car_status_set_IDLE } };
CANMSG_SpeedTypeDef              CANMSG_Speed          = { {0U, false}, { .encoder_l = 0, .encoder_r = 0, .inverter_l = 0, .inverter_r = 0 } };
CANMSG_HVVoltageTypeDef          CANMSG_HVVoltage      = { {0U, false}, { 0U } };
CANMSG_HVCurrentTypeDef          CANMSG_HVCurrent      = { {0U, false}, { 0U } };
CANMSG_HVTemperatureTypeDef      CANMSG_HVTemperature  = { {0U, false}, { 0U } };
CANMSG_HVErrorsTypeDef           CANMSG_HVErrors       = { {0U, false}, { 0U } };
CANMSG_HVFeedbacksTypeDef        CANMSG_HVFeedbacks    = { {0U, false}, { 0U } };
CANMSG_TSStatusTypeDef           CANMSG_TSStatus       = { {0U, false}, { .ts_status = primary_ts_status_ts_status_OFF } };
CANMSG_SetTSStatusTypeDef        CANMSG_SetTSStatus    = { {0U, false}, { .ts_status_set = primary_set_ts_status_das_ts_status_set_OFF } };
CANMSG_LVCurrentTypeDef          CANMSG_LVCurrent      = { {0U, false}, { 0U } };
CANMSG_LVVoltageTypeDef          CANMSG_LVVoltage      = { {0U, false}, { 0U } };
CANMSG_LVTemperatureTypeDef      CANMSG_LVTemperature  = { {0U, false}, { 0U } };
CANMSG_SetPedRangeTypeDef        CANMSG_SetPedRange    = { {0U, false}, { 0U } };
CANMSG_InvConnStatusTypeDef      CANMSG_InvConnStatus  = { {0U, false}, { 0U } };
CANMSG_SetPTTStatusTypeDef       CANMSG_SetPTTStatus   = { {0U, false}, { 0U } };
CANMSG_PTTStatusTypeDef          CANMSG_PTTStatus      = { {0U, false}, { .status = primary_ptt_status_status_OFF } };
CANMSG_SetInvConnStatusTypeDef   CANMSG_SetInvConnStatus = { {0U, false}, { 0U } };
CANMSG_SetSteerRangeTypeDef      CANMSG_SetSteerRange  = { {0U, false}, { 0U } };
CANMSG_AmbientTemperatureTypeDef CANMSG_AmbientTemperature  = { {0U, false}, { 0U } };

/* Secondary Network */
CANMSG_PedValsTypeDef        CANMSG_PedVals        = { {0U, false}, { 0U } };
CANMSG_CtrlOutTypeDef        CANMSG_CtrlOut        = { {0U, false}, { 0U } };
CANMSG_SteerValTypeDef       CANMSG_SteerVal       = { {0U, false}, { 0U } };
CANMSG_IMUAccTypeDef         CANMSG_IMUAcc         = { {0U, false}, { 0U } };
CANMSG_IMUAngTypeDef         CANMSG_IMUAng         = { {0U, false}, { 0U } };

CANFQ_QueueTypeDef _CANMSG_RX_queue = { 0U };



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
        if (inverters_id_is_message(msg.id)) {
            INV_parse_CAN_msg(msg.id, msg.data, msg.size);
        } else {
            _CANMSG_deserialize_msg_by_id(msg);

            CANMSG_MetadataTypeDef *msg_to_update = CANMSG_get_metadata_from_id(msg.id);
            if (msg_to_update != NULL) {
                msg_to_update->timestamp = HAL_GetTick();
                msg_to_update->is_new = true;
            }
        }
    }
}

CAN_HandleTypeDef* _CANMSG_get_nwk_from_id(CAN_IdTypeDef id) {
    switch (id) {
        case SECONDARY_PEDALS_OUTPUT_FRAME_ID:
        case PRIMARY_CONTROL_OUTPUT_FRAME_ID: // NOTE: forse da spostare
        case SECONDARY_STEERING_ANGLE_FRAME_ID:
        case SECONDARY_IMU_ACCELERATION_FRAME_ID:
        case SECONDARY_IMU_ANGULAR_RATE_FRAME_ID:
            return &CAN_SECONDARY_NETWORK;
        default:
            return &CAN_PRIMARY_NETWORK;
    }
}

void _CANMSG_deserialize_msg_by_id(CAN_MessageTypeDef msg) {
    switch (msg.id) {
        case PRIMARY_STEER_STATUS_FRAME_ID:
            primary_steer_status_unpack(&(CANMSG_SteerStatus.data), msg.data, PRIMARY_STEER_STATUS_BYTE_SIZE);
            break;
        case PRIMARY_SET_CAR_STATUS_FRAME_ID:
            primary_set_car_status_unpack(&(CANMSG_SetCarStatus.data), msg.data, PRIMARY_SET_CAR_STATUS_BYTE_SIZE);
            break;
        case PRIMARY_TS_STATUS_FRAME_ID:
            primary_ts_status_unpack(&(CANMSG_TSStatus.data), msg.data, PRIMARY_TS_STATUS_BYTE_SIZE);
            break;
        case PRIMARY_HV_VOLTAGE_FRAME_ID:
            primary_hv_voltage_unpack(&(CANMSG_HVVoltage.data), msg.data, PRIMARY_HV_VOLTAGE_BYTE_SIZE);
            break;
        case PRIMARY_HV_CURRENT_FRAME_ID:
            primary_hv_current_unpack(&(CANMSG_HVCurrent.data), msg.data, PRIMARY_HV_CURRENT_BYTE_SIZE);
            break;
        case PRIMARY_HV_TEMP_FRAME_ID:
            primary_hv_temp_unpack(&(CANMSG_HVTemperature.data), msg.data, PRIMARY_HV_TEMP_BYTE_SIZE);
            break;
        case PRIMARY_HV_ERRORS_FRAME_ID:
            primary_hv_errors_unpack(&(CANMSG_HVErrors.data), msg.data, PRIMARY_HV_ERRORS_BYTE_SIZE);
            break;
        case PRIMARY_HV_FEEDBACKS_STATUS_FRAME_ID:
            primary_hv_feedbacks_status_unpack(&(CANMSG_HVFeedbacks.data), msg.data, PRIMARY_HV_FEEDBACKS_STATUS_BYTE_SIZE);
            break;
        case PRIMARY_SET_PEDALS_RANGE_FRAME_ID:
            primary_set_pedals_range_unpack(&(CANMSG_SetPedRange.data), msg.data, PRIMARY_SET_PEDALS_RANGE_BYTE_SIZE);
            break;
        case PRIMARY_SET_STEERING_ANGLE_RANGE_FRAME_ID:
            primary_set_steering_angle_range_unpack(&(CANMSG_SetSteerRange.data), msg.data, PRIMARY_SET_STEERING_ANGLE_RANGE_BYTE_SIZE);
            break;
        case PRIMARY_INVERTER_CONNECTION_STATUS_FRAME_ID:
            primary_inverter_connection_status_unpack(&(CANMSG_InvConnStatus.data), msg.data, PRIMARY_INVERTER_CONNECTION_STATUS_BYTE_SIZE);
            break;
        case PRIMARY_SET_PTT_STATUS_FRAME_ID:
            primary_set_ptt_status_unpack(&(CANMSG_SetPTTStatus.data), msg.data, PRIMARY_SET_PTT_STATUS_BYTE_SIZE);
            break;
        case SECONDARY_IMU_ACCELERATION_FRAME_ID:
            secondary_imu_acceleration_t raw_imu_acc;
            secondary_imu_acceleration_unpack(&raw_imu_acc, msg.data, SECONDARY_IMU_ACCELERATION_BYTE_SIZE);
            secondary_imu_acceleration_raw_to_conversion_struct(&(CANMSG_IMUAcc.data), &raw_imu_acc);
            CANMSG_AmbientTemperature.data.temp = CANMSG_IMUAcc.data.temperature;
            CANMSG_AmbientTemperature.info.is_new = true;
            break;
        case SECONDARY_IMU_ANGULAR_RATE_FRAME_ID:
            secondary_imu_angular_rate_t raw_imu_ang;
            secondary_imu_angular_rate_unpack(&raw_imu_ang, msg.data, SECONDARY_IMU_ANGULAR_RATE_BYTE_SIZE);
            secondary_imu_angular_rate_raw_to_conversion_struct(&(CANMSG_IMUAng.data), &raw_imu_ang);
            break;
        default:
            // LOG_write(LOGLEVEL_ERR, "[CANMSG/Deserialize] Unknown message id: 0x%X", msg.id);
            break;
    }
}


/**
 * @brief     Return a pointer to the metadata struct for the specified CAN message
 */
CANMSG_MetadataTypeDef* CANMSG_get_metadata_from_id(CAN_IdTypeDef id) {
    switch(id) {
        case PRIMARY_DAS_VERSION_FRAME_ID:
            return &(CANMSG_DASVersion.info);
        case PRIMARY_DAS_ERRORS_FRAME_ID:
            return &(CANMSG_DASErrors.info);
        case PRIMARY_STEER_STATUS_FRAME_ID:
            return &(CANMSG_SteerStatus.info);
        case PRIMARY_CAR_STATUS_FRAME_ID:
            return &(CANMSG_CarStatus.info);
        case PRIMARY_SET_CAR_STATUS_FRAME_ID:
            return &(CANMSG_SetCarStatus.info);
        case PRIMARY_SPEED_FRAME_ID:
            return &(CANMSG_Speed.info);
        case PRIMARY_SET_TS_STATUS_DAS_FRAME_ID:
            return &(CANMSG_SetTSStatus.info);
        case PRIMARY_TS_STATUS_FRAME_ID:
            return &(CANMSG_TSStatus.info);
        case PRIMARY_HV_VOLTAGE_FRAME_ID:
            return &(CANMSG_HVVoltage.info);
        case PRIMARY_HV_CURRENT_FRAME_ID:
            return &(CANMSG_HVCurrent.info);
        case PRIMARY_HV_TEMP_FRAME_ID:
            return &(CANMSG_HVTemperature.info);
        case PRIMARY_HV_ERRORS_FRAME_ID:
            return &(CANMSG_HVErrors.info);
        case PRIMARY_HV_FEEDBACKS_STATUS_FRAME_ID:
            return &(CANMSG_HVFeedbacks.info);
        case PRIMARY_SET_PEDALS_RANGE_FRAME_ID:
            return &(CANMSG_SetPedRange.info);
        case PRIMARY_SET_STEERING_ANGLE_RANGE_FRAME_ID:
            return &(CANMSG_SetSteerRange.info);
        case PRIMARY_INVERTER_CONNECTION_STATUS_FRAME_ID:
            return &(CANMSG_InvConnStatus.info);
        case PRIMARY_SET_PTT_STATUS_FRAME_ID:
            return &(CANMSG_SetPTTStatus.info);
        case PRIMARY_PTT_STATUS_FRAME_ID:
            return &(CANMSG_PTTStatus.info);
        case PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID:
            return &(CANMSG_SetInvConnStatus.info);
        case PRIMARY_AMBIENT_TEMPERATURE_FRAME_ID:
            return &(CANMSG_AmbientTemperature.info);
        case PRIMARY_CONTROL_OUTPUT_FRAME_ID:
            return &(CANMSG_CtrlOut.info);
        // case SECONDARY_PEDALS_OUTPUT_FRAME_ID:
        //     return &(CANMSG_PedVals.info);
        // case SECONDARY_STEERING_ANGLE_FRAME_ID:
        //     return &(CANMSG_SteerVal.info);
        // case SECONDARY_IMU_ACCELERATION_FRAME_ID:
        //     return &(CANMSG_IMUAcc.info);
        // case SECONDARY_IMU_ANGULAR_RATE_FRAME_ID:
        //     return &(CANMSG_IMUAng.info);
        case PRIMARY_SET_CELL_BALANCING_STATUS_FRAME_ID:
        case PRIMARY_HANDCART_SETTINGS_SET_FRAME_ID:
        case PRIMARY_TLM_VERSION_FRAME_ID:
        case PRIMARY_TLM_STATUS_FRAME_ID:
            return NULL;
        default:
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata] Unknown message id: 0x%X", id);

            // uint8_t* name[30] = { 0U };
            // primary_message_name_from_id(id, name);
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata]     > primary nwk decoding: [%s]", name);
            // 
            // name[0] = '\0';
            // secondary_message_name_from_id(id, name);
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata]     > secondary nwk decoding: [%s]", name);
            // 
            // name[0] = '\0';
            // inverters_message_name_from_id(id, name);
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata]     > inverters nwk decoding: [%s]", name);
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
    static CAN_IdTypeDef ids_to_send[] = {
        // primary_ID_DAS_VERSION,
        PRIMARY_DAS_ERRORS_FRAME_ID,
        PRIMARY_CAR_STATUS_FRAME_ID,
        PRIMARY_SET_TS_STATUS_DAS_FRAME_ID,
        PRIMARY_SPEED_FRAME_ID,
        // PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID,
        // PRIMARY_AMBIENT_TEMPERATURE_FRAME_ID,
        PRIMARY_PTT_STATUS_FRAME_ID,

        // SECONDARY_PEDALS_OUTPUT_FRAME_ID,
        // PRIMARY_CONTROL_OUTPUT_FRAME_ID,
        // SECONDARY_STEERING_ANGLE_FRAME_ID
    };
    static uint8_t ids_loop_idx = 0;
    uint8_t ids_loop_len = sizeof(ids_to_send)/sizeof(ids_to_send[0]);

    /* Loop until either MBs are full or everything has been sent */
    for (uint8_t tx_count = 0; tx_count < ids_loop_len; tx_count++) {
        CAN_IdTypeDef id = ids_to_send[ids_loop_idx];
        CANMSG_MetadataTypeDef *info = CANMSG_get_metadata_from_id(id);
        CAN_HandleTypeDef *nwk = _CANMSG_get_nwk_from_id(id);

        /* Send if interval is elapsed or interval is "once" and msg is new */
        if (info != NULL && _CANMSG_needs_to_be_sent(id, nwk)) {
            CAN_MessageTypeDef msg;
            
            if (_CANMSG_serialize_msg_by_id(id, &msg)){
                if(CAN_send(&msg, nwk) != HAL_OK){
                    LOG_write(LOGLEVEL_ERR, "CAN SEND ERROR");
                }
            }
            else {
                LOG_write(LOGLEVEL_WARN, "[CANMSG/flushTX] Failed to serialize message 0x%X", id);
            }
            info->timestamp = HAL_GetTick();
            info->is_new = false;
        }

        ids_loop_idx = (ids_loop_idx + 1) % ids_loop_len;
    }
}

/**
 * @brief Check if a message needs to be sent. That is, if it is new and its interval
 *        is "once", or has elapsed since the last transmission.
 */
bool _CANMSG_needs_to_be_sent(CAN_IdTypeDef id, CAN_HandleTypeDef* nwk) {
    CANMSG_MetadataTypeDef *info = CANMSG_get_metadata_from_id(id);
    int32_t interval = (nwk == &CAN_PRIMARY_NETWORK) ? primary_watchdog_interval_from_id(id) : secondary_watchdog_interval_from_id(id);
    int32_t elapsed = HAL_GetTick() - info->timestamp;
    return (interval == -1 && info->is_new) ||
           (elapsed >= interval && interval != -1);
}

bool _CANMSG_serialize_msg_by_id(CAN_IdTypeDef id, CAN_MessageTypeDef *msg) {
    msg->id = id;
    
    switch (id) {
        // case primary_ID_DAS_VERSION:
        //     msg->size = primary_serialize_DAS_VERSION(msg->data, INT_COMPONENT_VERSION, can_BUILD_TIME);
        //     break;
        case PRIMARY_DAS_ERRORS_FRAME_ID:
            msg->size = primary_das_errors_pack(msg->data, &(CANMSG_DASErrors.data), PRIMARY_DAS_ERRORS_BYTE_SIZE);
            break;
        case PRIMARY_CAR_STATUS_FRAME_ID:
            msg->size = primary_car_status_pack(msg->data, &(CANMSG_CarStatus.data), PRIMARY_CAR_STATUS_BYTE_SIZE);
            break;
        case PRIMARY_SPEED_FRAME_ID:
            primary_speed_t speed = { 0U };
            primary_speed_conversion_to_raw_struct(&speed, &(CANMSG_Speed.data));
            msg->size = primary_speed_pack(msg->data, &speed, PRIMARY_SPEED_BYTE_SIZE);
            break;
        case PRIMARY_SET_TS_STATUS_DAS_FRAME_ID:
            msg->size = primary_set_ts_status_das_pack(msg->data, &(CANMSG_SetTSStatus.data), PRIMARY_SET_TS_STATUS_DAS_BYTE_SIZE);
            break;
        case PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID:
            msg->size = primary_set_inverter_connection_status_pack(msg->data, &(CANMSG_SetInvConnStatus.data), PRIMARY_SET_INVERTER_CONNECTION_STATUS_BYTE_SIZE);
            break;
        case PRIMARY_AMBIENT_TEMPERATURE_FRAME_ID:
            msg->size = primary_ambient_temperature_pack(
                msg->data,
                &(CANMSG_AmbientTemperature.data),
                PRIMARY_AMBIENT_TEMPERATURE_BYTE_SIZE
            );
            break;
        case PRIMARY_PTT_STATUS_FRAME_ID:
            msg->size = primary_ptt_status_pack(msg->data, &(CANMSG_PTTStatus.data), PRIMARY_PTT_STATUS_BYTE_SIZE);
            break;
        case SECONDARY_PEDALS_OUTPUT_FRAME_ID: ;
            secondary_pedals_output_t raw_ped;
            secondary_pedals_output_conversion_to_raw_struct(&raw_ped, &(CANMSG_PedVals.data));
            msg->size = secondary_pedals_output_pack(msg->data, &raw_ped, SECONDARY_PEDALS_OUTPUT_BYTE_SIZE);
            break;
        case PRIMARY_CONTROL_OUTPUT_FRAME_ID: ;
            primary_control_output_t raw_ctrl;
            primary_control_output_conversion_to_raw_struct(&raw_ctrl, &(CANMSG_CtrlOut.data));
            msg->size = primary_control_output_pack(msg->data, &raw_ctrl, PRIMARY_CONTROL_OUTPUT_BYTE_SIZE);
            break;
        // case SECONDARY_STEERING_ANGLE_FRAME_ID:
            // msg->size = secondary_steering_angle_pack(msg->data, &(CANMSG_SteerVal.data), SECONDARY_STEERING_ANGLE_BYTE_SIZE);
            // break;
        default:
            LOG_write(LOGLEVEL_ERR, "[CANMSG/Serialize] Unknown message id: 0x%X", msg->id); 

            uint8_t* name[30] = { 0U };
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
