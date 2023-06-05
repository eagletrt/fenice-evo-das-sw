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
// #include "inverters.h"
#include "string.h"

#include "../Lib/can/lib/primary/network.h"
#include "../Lib/can/lib/secondary/network.h"

#ifdef TESTING
    #include "../../tests/logger_stubs.h"
#else
    #include "logger.h"
#endif


/* Prototypes and inline functions ------------------------------------------ */
CAN_HandleTypeDef* _CANMSG_get_nwk_from_id(CAN_IdTypeDef);
void _CANMSG_deserialize_msg_by_id(CAN_MessageTypeDef);
size_t _CANMSG_parse_INV_L_message(CAN_MessageTypeDef*);
size_t _CANMSG_parse_INV_R_message(CAN_MessageTypeDef*);
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
CANMSG_HVErrorsTypeDef           CANMSG_HVErrors       = { {0U, false}, {0U}};
CANMSG_HVFeedbacksTypeDef        CANMSG_HVFeedbacks    = { {0U, false}, {0U} };
CANMSG_TSStatusTypeDef           CANMSG_TSStatus       = { {0U, false}, { .ts_status = primary_ts_status_ts_status_OFF } };
CANMSG_SetTSStatusTypeDef        CANMSG_SetTSStatus    = { {0U, false}, { .ts_status_set = primary_set_ts_status_ts_status_set_OFF } };
CANMSG_LVCurrentTypeDef          CANMSG_LVCurrent      = { {0U, false}, { 0U } };
CANMSG_LVVoltageTypeDef          CANMSG_LVVoltage      = { {0U, false}, { 0U } };
CANMSG_LVTemperatureTypeDef      CANMSG_LVTemperature  = { {0U, false}, { 0U } };
CANMSG_SetPedRangeTypeDef        CANMSG_SetPedRange    = { {0U, false}, { 0U } };
CANMSG_InvConnStatusTypeDef      CANMSG_InvConnStatus  = { {0U, false}, { 0U } };
CANMSG_SetInvConnStatusTypeDef   CANMSG_SetInvConnStatus = { {0U, false}, { 0U } };
CANMSG_SetSteerRangeTypeDef      CANMSG_SetSteerRange  = { {0U, false}, { 0U } };
CANMSG_AmbientTemperatureTypeDef CANMSG_AmbientTemperature  = { {0U, false}, { 0U } };

/* Primary Network - Inverters */
// CANMSG_Inv_SetTorqueTypeDef  CANMSG_InvL_SetTorque = { {0U, false}, { .data_0 = INV_REG_TORQUECMD, .data_1 = 0U, .data_2 = 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_Status    = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_IOInfo    = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_Errors    = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_Speed     = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_MTemp     = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_ITemp     = { {0U, false}, { 0U } };
// CANMSG_Inv_SetTorqueTypeDef  CANMSG_InvR_SetTorque = { {0U, false}, { .data_0 = INV_REG_TORQUECMD, .data_1 = 0U, .data_2 = 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_Status    = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_IOInfo    = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_Errors    = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_Speed     = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_MTemp     = { {0U, false}, { 0U } };
// CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_ITemp     = { {0U, false}, { 0U } };

/* Secondary Network */
CANMSG_PedValsTypeDef        CANMSG_PedVals        = { {0U, false}, { 0U } };
CANMSG_CtrlOutTypeDef        CANMSG_CtrlOut        = { {0U, false}, { 0U } };
CANMSG_SteerValTypeDef       CANMSG_SteerVal       = { {0U, false}, { 0U } };
CANMSG_IMUAccTypeDef         CANMSG_IMUAcc         = { {0U, false}, { 0U } };
CANMSG_IMUAngTypeDef         CANMSG_IMUAng         = { {0U, false}, { 0U } };


void CANMSG_process_RX(CAN_MessageTypeDef msg) {
    _CANMSG_deserialize_msg_by_id(msg);

    // if (msg.id != primary_ID_INV_L_RESPONSE && msg.id != primary_ID_INV_R_RESPONSE) {
        CANMSG_MetadataTypeDef *msg_to_update = CANMSG_get_metadata_from_id(msg.id);
        if (msg_to_update != NULL) {
            msg_to_update->timestamp = HAL_GetTick();
            msg_to_update->is_new = true;
        }
    // }
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
            primary_steer_status_unpack(&(CANMSG_SteerStatus.data), msg.data, PRIMARY_STEER_STATUS_LENGTH);
            break;
        case PRIMARY_SET_CAR_STATUS_FRAME_ID:
            primary_set_car_status_unpack(&(CANMSG_SetCarStatus.data), msg.data, PRIMARY_SET_CAR_STATUS_LENGTH);
            break;
        case PRIMARY_TS_STATUS_FRAME_ID:
            primary_ts_status_unpack(&(CANMSG_TSStatus.data), msg.data, PRIMARY_TS_STATUS_LENGTH);
            break;
        case PRIMARY_HV_VOLTAGE_FRAME_ID:
            primary_hv_voltage_unpack(&(CANMSG_HVVoltage.data), msg.data, PRIMARY_HV_VOLTAGE_LENGTH);
            break;
        case PRIMARY_HV_CURRENT_FRAME_ID:
            primary_hv_current_unpack(&(CANMSG_HVCurrent.data), msg.data, PRIMARY_HV_CURRENT_LENGTH);
            break;
        case PRIMARY_HV_TEMP_FRAME_ID:
            primary_hv_temp_unpack(&(CANMSG_HVTemperature.data), msg.data, PRIMARY_HV_TEMP_LENGTH);
            break;
        case PRIMARY_HV_ERRORS_FRAME_ID:
            primary_hv_errors_unpack(&(CANMSG_HVErrors.data), msg.data, PRIMARY_HV_ERRORS_LENGTH);
            break;
        case PRIMARY_HV_FEEDBACKS_STATUS_FRAME_ID:
            primary_hv_feedbacks_status_unpack(&(CANMSG_HVFeedbacks.data), msg.data, PRIMARY_HV_FEEDBACKS_STATUS_LENGTH);
            break;
        case PRIMARY_SET_PEDALS_RANGE_FRAME_ID:
            primary_set_pedals_range_unpack(&(CANMSG_SetPedRange.data), msg.data, PRIMARY_SET_PEDALS_RANGE_LENGTH);
            break;
        case PRIMARY_SET_STEERING_ANGLE_RANGE_FRAME_ID:
            primary_set_steering_angle_range_unpack(&(CANMSG_SetSteerRange.data), msg.data, PRIMARY_SET_STEERING_ANGLE_RANGE_LENGTH);
            break;
        case PRIMARY_INVERTER_CONNECTION_STATUS_FRAME_ID:
            primary_inverter_connection_status_unpack(&(CANMSG_InvConnStatus.data), msg.data, PRIMARY_INVERTER_CONNECTION_STATUS_LENGTH);
            break;
        case PRIMARY_INV_L_RESPONSE_FRAME_ID:
            _CANMSG_parse_INV_L_message(&msg);
            break;
        case PRIMARY_INV_R_RESPONSE_FRAME_ID:
            _CANMSG_parse_INV_R_message(&msg);
            break;
        case SECONDARY_IMU_ACCELERATION_FRAME_ID:
            secondary_imu_acceleration_t raw_imu_acc;
            secondary_imu_acceleration_unpack(&raw_imu_acc, msg.data, SECONDARY_IMU_ACCELERATION_LENGTH);
            secondary_imu_acceleration_raw_to_conversion_struct(&(CANMSG_IMUAcc.data), &raw_imu_acc);
            CANMSG_AmbientTemperature.data.temp = CANMSG_IMUAcc.data.temperature;
            CANMSG_AmbientTemperature.info.is_new = true;
            break;
        case SECONDARY_IMU_ANGULAR_RATE_FRAME_ID:
            secondary_imu_angular_rate_t raw_imu_ang;
            secondary_imu_angular_rate_unpack(&raw_imu_ang, msg.data, SECONDARY_IMU_ANGULAR_RATE_LENGTH);
            secondary_imu_angular_rate_raw_to_conversion_struct(&(CANMSG_IMUAng.data), &raw_imu_ang);
            break;
        default:
            // LOG_write(LOGLEVEL_ERR, "[CANMSG/Deserialize] Unknown message id: 0x%X", msg.id);
            break;
    }
}

size_t _CANMSG_parse_INV_L_message(CAN_MessageTypeDef *msg) {
    primary_inv_l_response_t inv_response;
    primary_inv_l_response_unpack(&inv_response, msg->data, PRIMARY_INV_L_RESPONSE_LENGTH);
    CANMSG_MetadataTypeDef *info = NULL;

    // switch (inv_response.reg_id) {
    //     case INV_REG_STATUS:
    //         // LOG_write(LOGLEVEL_DEBUG, "INV/L/Status msg");
    //         memcpy(&(CANMSG_InvL_Status.data), &(inv_response), 5);
    //         info = &(CANMSG_InvL_Status.info);
    //         break;
    //     case INV_REG_IOINFO:
    //         // LOG_write(LOGLEVEL_DEBUG, "INV/L/IOInfo msg");
    //         memcpy(&(CANMSG_InvL_IOInfo.data), &(inv_response), 5);
    //         info = &(CANMSG_InvL_IOInfo.info);
    //         break;
    //     case INV_REG_ERRORS:
    //         // LOG_write(LOGLEVEL_DEBUG, "INV/L/Errs msg");
    //         memcpy(&(CANMSG_InvL_Errors.data), &(inv_response), 5);
    //         info = &(CANMSG_InvL_Errors.info);
    //         break;
    //     case INV_REG_SPEED:
    //         // LOG_write(LOGLEVEL_DEBUG, "INV/L/Speed msg");
    //         memcpy(&(CANMSG_InvL_Speed.data), &(inv_response), 3);
    //         info = &(CANMSG_InvL_Speed.info);
    //         break;
    //     case INV_REG_MOT_TEMP:
    //         memcpy(&(CANMSG_InvL_MTemp.data), &(inv_response), 3);
    //         info = &(CANMSG_InvL_MTemp.info);
    //         break;
    //     case INV_REG_INV_TEMP:
    //         memcpy(&(CANMSG_InvL_ITemp.data), &(inv_response), 3);
    //         info = &(CANMSG_InvL_ITemp.info);
    //         break;
    //     case INV_REG_I_ACTUAL:
    //     case INV_REG_I_CMD:
    //     case INV_REG_I_CMD_RAMP:
    //         break;
    //     default:
    //         LOG_write(LOGLEVEL_WARN, "[CANMSG/deserialize] Unkown L inverter REG_ID: 0x%X", msg->data[0]);
    //         break;
    // }

    if (info) {
        info->timestamp = HAL_GetTick();
        info->is_new = true;
    }

    return 0U;
}

size_t _CANMSG_parse_INV_R_message(CAN_MessageTypeDef *msg) {
    primary_inv_r_response_t inv_response;
    primary_inv_r_response_unpack(&inv_response, msg->data, PRIMARY_INV_R_RESPONSE_LENGTH);
    CANMSG_MetadataTypeDef *info = NULL;

    // switch (inv_response.reg_id) {
    //     case INV_REG_STATUS:
    //         // LOG_write(LOGLEVEL_DEBUG, "INV/R/Status msg");
    //         memcpy(&(CANMSG_InvR_Status.data), &(inv_response), 5);
    //         info = &(CANMSG_InvR_Status.info);
    //         break;
    //     case INV_REG_IOINFO:
    //         // LOG_write(LOGLEVEL_DEBUG, "INV/R/IOInfo msg");
    //         memcpy(&(CANMSG_InvR_IOInfo.data), &(inv_response), 5);
    //         info = &(CANMSG_InvR_IOInfo.info);
    //         break;
    //     case INV_REG_ERRORS:
    //         // LOG_write(LOGLEVEL_DEBUG, "INV/R/Errs msg");
    //         memcpy(&(CANMSG_InvR_Errors.data), &(inv_response), 5);
    //         info = &(CANMSG_InvR_Errors.info);
    //         break;
    //     case INV_REG_SPEED:
    //         // LOG_write(LOGLEVEL_DEBUG, "INV/R/Speed msg");
    //         memcpy(&(CANMSG_InvR_Speed.data), &(inv_response), 3);
    //         info = &(CANMSG_InvR_Speed.info);
    //         break;
    //     case INV_REG_MOT_TEMP:
    //         memcpy(&(CANMSG_InvR_MTemp.data), &(inv_response), 3);
    //         info = &(CANMSG_InvR_MTemp.info);
    //         break;
    //     case INV_REG_INV_TEMP:
    //         memcpy(&(CANMSG_InvR_ITemp.data), &(inv_response), 3);
    //         info = &(CANMSG_InvR_ITemp.info);
    //         break;
    //     case INV_REG_I_ACTUAL:
    //     case INV_REG_I_CMD:
    //     case INV_REG_I_CMD_RAMP:
    //         break;
    //     default:
    //         LOG_write(LOGLEVEL_WARN, "[CANMSG/deserialize] Unkown R inverter REG_ID: 0x%X", msg->data[0]);
    //         break;
    // }

    if (info) {
        info->timestamp = HAL_GetTick();
        info->is_new = true;
    }

    return 0U;
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
        case PRIMARY_SET_TS_STATUS_FRAME_ID:
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
        case PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID:
            return &(CANMSG_SetInvConnStatus.info);
        case PRIMARY_INV_L_REQUEST_FRAME_ID:
            // return &(CANMSG_InvL_SetTorque.info);
        case PRIMARY_INV_R_REQUEST_FRAME_ID:
            // return &(CANMSG_InvR_SetTorque.info);
        case PRIMARY_AMBIENT_TEMPERATURE_FRAME_ID:
            return &(CANMSG_AmbientTemperature.info);
        case SECONDARY_PEDALS_OUTPUT_FRAME_ID:
            return &(CANMSG_PedVals.info);
        case PRIMARY_CONTROL_OUTPUT_FRAME_ID:
            return &(CANMSG_CtrlOut.info);
        case SECONDARY_STEERING_ANGLE_FRAME_ID:
            return &(CANMSG_SteerVal.info);
        case SECONDARY_IMU_ACCELERATION_FRAME_ID:
            return &(CANMSG_IMUAcc.info);
        case SECONDARY_IMU_ANGULAR_RATE_FRAME_ID:
            return &(CANMSG_IMUAng.info);
        default:
            // LOG_write(LOGLEVEL_WARN, "[CANMSG/getMetadata] Unknown message id: 0x%X", id);
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
        PRIMARY_SET_TS_STATUS_FRAME_ID,
        PRIMARY_SPEED_FRAME_ID,
        PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID,
        PRIMARY_INV_L_REQUEST_FRAME_ID,
        PRIMARY_INV_R_REQUEST_FRAME_ID,
        PRIMARY_AMBIENT_TEMPERATURE_FRAME_ID,

        SECONDARY_PEDALS_OUTPUT_FRAME_ID,
        PRIMARY_CONTROL_OUTPUT_FRAME_ID,
        SECONDARY_STEERING_ANGLE_FRAME_ID
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
            
            if (_CANMSG_serialize_msg_by_id(id, &msg))
                CAN_send(&msg, nwk);
            else
                LOG_write(LOGLEVEL_WARN, "[CANMSG/flushTX] Failed to serialize message 0x%X", id);

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
    // int32_t interval = (nwk == &CAN_PRIMARY_NETWORK) ? primary_watchdog_interval_from_id(id) : secondary_watchdog_interval_from_id(id);
    // int32_t elapsed = HAL_GetTick() - info->timestamp;
    // return (interval == primary_INTERVAL_ONCE && info->is_new) ||
    //        (elapsed >= interval && interval != primary_INTERVAL_ONCE);
    return true;
}

bool _CANMSG_serialize_msg_by_id(CAN_IdTypeDef id, CAN_MessageTypeDef *msg) {
    msg->id = id;
    
    switch (id) {
        // case primary_ID_DAS_VERSION:
        //     msg->size = primary_serialize_DAS_VERSION(msg->data, INT_COMPONENT_VERSION, can_BUILD_TIME);
        //     break;
        case PRIMARY_DAS_ERRORS_FRAME_ID:
            msg->size = primary_das_errors_pack(msg->data, CANMSG_DASErrors.data.das_error, PRIMARY_DAS_ERRORS_LENGTH);
            break;
        case PRIMARY_CAR_STATUS_FRAME_ID:
            msg->size = primary_car_status_pack(msg->data, &(CANMSG_CarStatus.data), PRIMARY_CAR_STATUS_LENGTH);
            break;
        case PRIMARY_SPEED_FRAME_ID:
            primary_speed_t speed = { 0U };
            primary_speed_conversion_to_raw_struct(&speed, &(CANMSG_Speed.data));
            msg->size = primary_speed_pack(msg->data, &speed, PRIMARY_SPEED_LENGTH);
            break;
        case PRIMARY_SET_TS_STATUS_FRAME_ID:
            msg->size = primary_set_ts_status_pack(msg->data, &(CANMSG_SetTSStatus.data), PRIMARY_SET_TS_STATUS_LENGTH);
            break;
        case PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID:
            msg->size = primary_set_inverter_connection_status_pack(msg->data, CANMSG_SetInvConnStatus.data.status, PRIMARY_SET_INVERTER_CONNECTION_STATUS_LENGTH);
            break;
        case PRIMARY_INV_L_REQUEST_FRAME_ID:
            // msg->size = primary_serialize_INV_L_REQUEST(
            //     msg->data,
            //     CANMSG_InvL_SetTorque.data.data_0,
            //     CANMSG_InvL_SetTorque.data.data_1,
            //     CANMSG_InvL_SetTorque.data.data_2,
            //     0, 0, 0, 0, 0
            // );
            msg->size = 3;
            break;
        case PRIMARY_INV_R_REQUEST_FRAME_ID:
            // msg->size = primary_serialize_INV_R_REQUEST(
            //     msg->data,
            //     CANMSG_InvR_SetTorque.data.data_0,
            //     CANMSG_InvR_SetTorque.data.data_1,
            //     CANMSG_InvR_SetTorque.data.data_2,
            //     0, 0, 0, 0, 0
            // );
            msg->size = 3;
            break;
        case PRIMARY_AMBIENT_TEMPERATURE_FRAME_ID:
            msg->size = primary_serialize_AMBIENT_TEMPERATURE(
                msg->data,
                CANMSG_AmbientTemperature.data.temp
            );
            break;
        case SECONDARY_PEDALS_OUTPUT_FRAME_ID: ;
            secondary_pedals_output_t raw_ped;
            secondary_pedals_output_conversion_to_raw_struct(&raw_ped, &(CANMSG_PedVals.data));
            msg->size = secondary_pedals_output_pack(msg->data, &raw_ped, SECONDARY_PEDALS_OUTPUT_LENGTH);
            break;
        case PRIMARY_CONTROL_OUTPUT_FRAME_ID: ;
            primary_control_output_t raw_ctrl;
            primary_control_output_conversion_to_raw_struct(&raw_ctrl, &(CANMSG_CtrlOut.data));
            msg->size = primary_control_output_pack(msg->data, &raw_ctrl, PRIMARY_CONTROL_OUTPUT_LENGTH);
            break;
        case SECONDARY_STEERING_ANGLE_FRAME_ID:
            msg->size = secondary_steering_angle_pack(msg->data, CANMSG_SteerVal.data.angle, SECONDARY_STEERING_ANGLE_LENGTH);
            break;
        default:
            LOG_write(LOGLEVEL_ERR, "[CANMSG/Serialize] Unknown message id: 0x%X", msg->id); 
            return false;
            break;
    }

    return true;
}
