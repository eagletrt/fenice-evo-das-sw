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

#include "../Lib/can/lib/primary/c/ids.h"
#include "../Lib/can/lib/primary/c/network.h"
#include "../Lib/can/lib/primary/c/watchdog.h"
#include "../Lib/can/lib/secondary/c/ids.h"
#include "../Lib/can/lib/secondary/c/network.h"
#include "../Lib/can/lib/secondary/c/watchdog.h"

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
CANMSG_CarStatusTypeDef          CANMSG_CarStatus      = { {0U, false}, { .car_status = primary_CarStatus_IDLE } };
CANMSG_SetCarStatusTypeDef       CANMSG_SetCarStatus   = { {0U, false}, { .car_status_set = primary_SetCarStatus_IDLE } };
CANMSG_SpeedTypeDef              CANMSG_Speed          = { {0U, false}, { .encoder_l = 0, .encoder_r = 0, .inverter_l = 0, .inverter_r = 0 } };
CANMSG_HVVoltageTypeDef          CANMSG_HVVoltage      = { {0U, false}, { 0U } };
CANMSG_HVCurrentTypeDef          CANMSG_HVCurrent      = { {0U, false}, { 0U } };
CANMSG_HVTemperatureTypeDef      CANMSG_HVTemperature  = { {0U, false}, { 0U } };
CANMSG_HVErrorsTypeDef           CANMSG_HVErrors       = { {0U, false}, { .errors = 0U, .warnings = 0U } };
CANMSG_HVFeedbacksTypeDef        CANMSG_HVFeedbacks    = { {0U, false}, { .feedbacks_status = 0U, .is_circuitry_error = 0U } };
CANMSG_TSStatusTypeDef           CANMSG_TSStatus       = { {0U, false}, { .ts_status = primary_TsStatus_OFF } };
CANMSG_SetTSStatusTypeDef        CANMSG_SetTSStatus    = { {0U, false}, { .ts_status_set = primary_Toggle_OFF } };
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
CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_Status    = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_IOInfo    = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_Errors    = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_Speed     = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_MTemp     = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvL_ITemp     = { {0U, false}, { 0U } };
// CANMSG_Inv_SetTorqueTypeDef  CANMSG_InvR_SetTorque = { {0U, false}, { .data_0 = INV_REG_TORQUECMD, .data_1 = 0U, .data_2 = 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_Status    = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_IOInfo    = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_Errors    = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_Speed     = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_MTemp     = { {0U, false}, { 0U } };
CANMSG_Inv_ResponseTypeDef   CANMSG_InvR_ITemp     = { {0U, false}, { 0U } };

/* Secondary Network */
CANMSG_PedValsTypeDef        CANMSG_PedVals        = { {0U, false}, { 0U } };
CANMSG_CtrlOutTypeDef        CANMSG_CtrlOut        = { {0U, false}, { 0U } };
CANMSG_SteerValTypeDef       CANMSG_SteerVal       = { {0U, false}, { 0U } };
CANMSG_IMUAccTypeDef         CANMSG_IMUAcc         = { {0U, false}, { 0U } };
CANMSG_IMUAngTypeDef         CANMSG_IMUAng         = { {0U, false}, { 0U } };


void CANMSG_process_RX(CAN_MessageTypeDef msg) {
    _CANMSG_deserialize_msg_by_id(msg);

    if (msg.id != primary_ID_INV_L_RESPONSE && msg.id != primary_ID_INV_R_RESPONSE) {
        CANMSG_MetadataTypeDef *msg_to_update = CANMSG_get_metadata_from_id(msg.id);
        if (msg_to_update != NULL) {
            msg_to_update->timestamp = HAL_GetTick();
            msg_to_update->is_new = true;
        }
    }
}

CAN_HandleTypeDef* _CANMSG_get_nwk_from_id(CAN_IdTypeDef id) {
    switch (id) {
        case secondary_ID_PEDALS_OUTPUT:
        case primary_ID_CONTROL_OUTPUT: // NOTE: forse da spostare
        case secondary_ID_STEERING_ANGLE:
        case secondary_ID_IMU_ACCELERATION:
        case secondary_ID_IMU_ANGULAR_RATE:
            return &CAN_SECONDARY_NETWORK;
        default:
            return &CAN_PRIMARY_NETWORK;
    }
}

void _CANMSG_deserialize_msg_by_id(CAN_MessageTypeDef msg) {
    switch (msg.id) {
        case primary_ID_STEER_STATUS:
            primary_deserialize_STEER_STATUS(&(CANMSG_SteerStatus.data), msg.data);
            break;
        case primary_ID_SET_CAR_STATUS:
            primary_deserialize_SET_CAR_STATUS(&(CANMSG_SetCarStatus.data), msg.data);
            break;
        case primary_ID_TS_STATUS:
            primary_deserialize_TS_STATUS(&(CANMSG_TSStatus.data), msg.data);
            break;
        case primary_ID_HV_VOLTAGE:
            primary_deserialize_HV_VOLTAGE(&(CANMSG_HVVoltage.data), msg.data);
            break;
        case primary_ID_HV_CURRENT:
            primary_deserialize_HV_CURRENT(&(CANMSG_HVCurrent.data), msg.data);
            break;
        case primary_ID_HV_TEMP:
            primary_deserialize_HV_TEMP(&(CANMSG_HVTemperature.data), msg.data);
            break;
        case primary_ID_HV_ERRORS:
            primary_deserialize_HV_ERRORS(&(CANMSG_HVErrors.data), msg.data);
            break;
        case primary_ID_HV_FEEDBACKS_STATUS:
            primary_deserialize_HV_FEEDBACKS_STATUS(&(CANMSG_HVFeedbacks.data), msg.data);
            break;
        case primary_ID_SET_PEDALS_RANGE:
            primary_deserialize_SET_PEDALS_RANGE(&(CANMSG_SetPedRange.data), msg.data);
            break;
        case primary_ID_SET_STEERING_ANGLE_RANGE:
            primary_deserialize_SET_STEERING_ANGLE_RANGE(&(CANMSG_SetSteerRange.data), msg.data);
            break;
        case primary_ID_INVERTER_CONNECTION_STATUS:
            primary_deserialize_INVERTER_CONNECTION_STATUS(&(CANMSG_InvConnStatus.data), msg.data);
            break;
        case primary_ID_INV_L_RESPONSE:
            _CANMSG_parse_INV_L_message(&msg);
            break;
        case primary_ID_INV_R_RESPONSE:
            _CANMSG_parse_INV_R_message(&msg);
            break;
        case secondary_ID_IMU_ACCELERATION:
            secondary_message_IMU_ACCELERATION raw_imu_acc;
            secondary_deserialize_IMU_ACCELERATION(&raw_imu_acc, msg.data);
            secondary_raw_to_conversion_struct_IMU_ACCELERATION(&(CANMSG_IMUAcc.data), &raw_imu_acc);
            CANMSG_AmbientTemperature.data.temp = CANMSG_IMUAcc.data.temperature;
            CANMSG_AmbientTemperature.info.is_new = true;
            break;
        case secondary_ID_IMU_ANGULAR_RATE:
            secondary_message_IMU_ANGULAR_RATE raw_imu_ang;
            secondary_deserialize_IMU_ANGULAR_RATE(&raw_imu_ang, msg.data);
            secondary_raw_to_conversion_struct_IMU_ANGULAR_RATE(&(CANMSG_IMUAng.data), &raw_imu_ang);
            break;
        default:
            // LOG_write(LOGLEVEL_ERR, "[CANMSG/Deserialize] Unknown message id: 0x%X", msg.id);
            break;
    }
}

size_t _CANMSG_parse_INV_L_message(CAN_MessageTypeDef *msg) {
    primary_message_INV_L_RESPONSE inv_response;
    primary_deserialize_INV_L_RESPONSE(&inv_response, msg->data);
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
    primary_message_INV_R_RESPONSE inv_response;
    primary_deserialize_INV_R_RESPONSE(&inv_response, msg->data);
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
        case primary_ID_DAS_VERSION:
            return &(CANMSG_DASVersion.info);
        case primary_ID_DAS_ERRORS:
            return &(CANMSG_DASErrors.info);
        case primary_ID_STEER_STATUS:
            return &(CANMSG_SteerStatus.info);
        case primary_ID_CAR_STATUS:
            return &(CANMSG_CarStatus.info);
        case primary_ID_SET_CAR_STATUS:
            return &(CANMSG_SetCarStatus.info);
        case primary_ID_SPEED:
            return &(CANMSG_Speed.info);
        case primary_ID_SET_TS_STATUS_DAS:
            return &(CANMSG_SetTSStatus.info);
        case primary_ID_TS_STATUS:
            return &(CANMSG_TSStatus.info);
        case primary_ID_HV_VOLTAGE:
            return &(CANMSG_HVVoltage.info);
        case primary_ID_HV_CURRENT:
            return &(CANMSG_HVCurrent.info);
        case primary_ID_HV_TEMP:
            return &(CANMSG_HVTemperature.info);
        case primary_ID_HV_ERRORS:
            return &(CANMSG_HVErrors.info);
        case primary_ID_HV_FEEDBACKS_STATUS:
            return &(CANMSG_HVFeedbacks.info);
        case primary_ID_SET_PEDALS_RANGE:
            return &(CANMSG_SetPedRange.info);
        case primary_ID_SET_STEERING_ANGLE_RANGE:
            return &(CANMSG_SetSteerRange.info);
        case primary_ID_INVERTER_CONNECTION_STATUS:
            return &(CANMSG_InvConnStatus.info);
        case primary_ID_SET_INVERTER_CONNECTION_STATUS:
            return &(CANMSG_SetInvConnStatus.info);
        case primary_ID_INV_L_REQUEST:
            // return &(CANMSG_InvL_SetTorque.info);
        case primary_ID_INV_R_REQUEST:
            // return &(CANMSG_InvR_SetTorque.info);
        case primary_ID_AMBIENT_TEMPERATURE:
            return &(CANMSG_AmbientTemperature.info);
        case secondary_ID_PEDALS_OUTPUT:
            return &(CANMSG_PedVals.info);
        case primary_ID_CONTROL_OUTPUT:
            return &(CANMSG_CtrlOut.info);
        case secondary_ID_STEERING_ANGLE:
            return &(CANMSG_SteerVal.info);
        case secondary_ID_IMU_ACCELERATION:
            return &(CANMSG_IMUAcc.info);
        case secondary_ID_IMU_ANGULAR_RATE:
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
        primary_ID_DAS_VERSION,
        primary_ID_DAS_ERRORS,
        primary_ID_CAR_STATUS,
        primary_ID_SET_TS_STATUS_DAS,
        primary_ID_SPEED,
        primary_ID_SET_INVERTER_CONNECTION_STATUS,
        primary_ID_INV_L_REQUEST,
        primary_ID_INV_R_REQUEST,
        primary_ID_AMBIENT_TEMPERATURE,

        secondary_ID_PEDALS_OUTPUT,
        primary_ID_CONTROL_OUTPUT,
        secondary_ID_STEERING_ANGLE
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
    int32_t interval = (nwk == &CAN_PRIMARY_NETWORK) ? primary_watchdog_interval_from_id(id) : secondary_watchdog_interval_from_id(id);
    int32_t elapsed = HAL_GetTick() - info->timestamp;
    return (interval == primary_INTERVAL_ONCE && info->is_new) ||
           (elapsed >= interval && interval != primary_INTERVAL_ONCE);
}

bool _CANMSG_serialize_msg_by_id(CAN_IdTypeDef id, CAN_MessageTypeDef *msg) {
    msg->id = id;
    
    switch (id) {
        // case primary_ID_DAS_VERSION:
        //     msg->size = primary_serialize_DAS_VERSION(msg->data, INT_COMPONENT_VERSION, can_BUILD_TIME);
        //     break;
        case primary_ID_DAS_ERRORS:
            msg->size = primary_serialize_DAS_ERRORS(msg->data, CANMSG_DASErrors.data.das_error);
            break;
        case primary_ID_CAR_STATUS:
            msg->size = primary_serialize_struct_CAR_STATUS(msg->data, &(CANMSG_CarStatus.data));
            break;
        case primary_ID_SPEED:
            primary_message_SPEED speed = { 0U };
            primary_conversion_to_raw_struct_SPEED(&speed, &(CANMSG_Speed.data));
            msg->size = primary_serialize_struct_SPEED(msg->data, &speed);
            break;
        case primary_ID_SET_TS_STATUS_DAS:
            msg->size = primary_serialize_struct_SET_TS_STATUS(msg->data, &(CANMSG_SetTSStatus.data));
            break;
        case primary_ID_SET_INVERTER_CONNECTION_STATUS:
            msg->size = primary_serialize_SET_INVERTER_CONNECTION_STATUS(msg->data, CANMSG_SetInvConnStatus.data.status);
            break;
        case primary_ID_INV_L_REQUEST:
            // msg->size = primary_serialize_INV_L_REQUEST(
            //     msg->data,
            //     CANMSG_InvL_SetTorque.data.data_0,
            //     CANMSG_InvL_SetTorque.data.data_1,
            //     CANMSG_InvL_SetTorque.data.data_2,
            //     0, 0, 0, 0, 0
            // );
            msg->size = 3;
            break;
        case primary_ID_INV_R_REQUEST:
            // msg->size = primary_serialize_INV_R_REQUEST(
            //     msg->data,
            //     CANMSG_InvR_SetTorque.data.data_0,
            //     CANMSG_InvR_SetTorque.data.data_1,
            //     CANMSG_InvR_SetTorque.data.data_2,
            //     0, 0, 0, 0, 0
            // );
            msg->size = 3;
            break;
        case primary_ID_AMBIENT_TEMPERATURE:
            msg->size = primary_serialize_AMBIENT_TEMPERATURE(
                msg->data,
                CANMSG_AmbientTemperature.data.temp
            );
            break;
        case secondary_ID_PEDALS_OUTPUT: ;
            secondary_message_PEDALS_OUTPUT raw_ped;
            secondary_conversion_to_raw_struct_PEDALS_OUTPUT(&raw_ped, &(CANMSG_PedVals.data));
            msg->size = secondary_serialize_struct_PEDALS_OUTPUT(msg->data, &raw_ped);
            break;
        case primary_ID_CONTROL_OUTPUT: ;
            primary_message_CONTROL_OUTPUT raw_ctrl;
            primary_conversion_to_raw_struct_CONTROL_OUTPUT(&raw_ctrl, &(CANMSG_CtrlOut.data));
            msg->size = primary_serialize_struct_CONTROL_OUTPUT(msg->data, &raw_ctrl);
            break;
        case secondary_ID_STEERING_ANGLE:
            msg->size = secondary_serialize_STEERING_ANGLE(msg->data, CANMSG_SteerVal.data.angle);
            break;
        default:
            LOG_write(LOGLEVEL_ERR, "[CANMSG/Serialize] Unknown message id: 0x%X", msg->id); 
            return false;
            break;
    }

    return true;
}
