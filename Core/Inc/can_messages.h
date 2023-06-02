#ifndef CANMSG_H
#define CANMSG_H

#include "can.h"
#include "stdbool.h"
#include "../Lib/can/lib/primary/network.h"
#include "../Lib/can/lib/secondary/network.h"


typedef struct {
    uint32_t timestamp;     /*< Timestamp of when the message was received/sent */
    bool is_new;            /*< If the message data has yet to be processed     */
} CANMSG_MetadataTypeDef;


/* Primary Network */
typedef struct { CANMSG_MetadataTypeDef info; primary_das_version_t data;      } CANMSG_DASVersionTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_das_errors_t data;       } CANMSG_DASErrorsTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_steer_status_t data;     } CANMSG_SteerStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_car_status_t data;       } CANMSG_CarStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_car_status_t data;   } CANMSG_SetCarStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_speed_converted_t data; } CANMSG_SpeedTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_voltage_t data;       } CANMSG_HVVoltageTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_current_t data;       } CANMSG_HVCurrentTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_temp_t data;          } CANMSG_HVTemperatureTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_errors_converted_t data;        } CANMSG_HVErrorsTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_feedbacks_status_t data; } CANMSG_HVFeedbacksTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_ts_status_t data;        } CANMSG_TSStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_ts_status_t data;    } CANMSG_SetTSStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_lv_current_t data;       } CANMSG_LVCurrentTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_lv_voltage_t data;       } CANMSG_LVVoltageTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_lv_temperature_t data;   } CANMSG_LVTemperatureTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_inverter_connection_status_t data;     } CANMSG_InvConnStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_inverter_connection_status_t data; } CANMSG_SetInvConnStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_pedals_range_t data; } CANMSG_SetPedRangeTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_steering_angle_range_t data; } CANMSG_SetSteerRangeTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_ambient_temperature_t data; } CANMSG_AmbientTemperatureTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_control_output_converted_t data; } CANMSG_CtrlOutTypeDef;

/* Primary Network - Inverters */
// typedef struct { CANMSG_MetadataTypeDef info; primary_message_INV_L_REQUEST data;    } CANMSG_Inv_SetTorqueTypeDef;
// typedef struct { CANMSG_MetadataTypeDef info; primary_message_INV_L_RESPONSE data;   } CANMSG_Inv_ResponseTypeDef;

/* Secondary Network */
typedef struct { CANMSG_MetadataTypeDef info; secondary_pedals_output_converted_t data;  } CANMSG_PedValsTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; secondary_steering_angle_t data; } CANMSG_SteerValTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; secondary_imu_acceleration_converted_t data; } CANMSG_IMUAccTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; secondary_imu_angular_rate_converted_t data; } CANMSG_IMUAngTypeDef;


extern CANMSG_DASVersionTypeDef       CANMSG_DASVersion;
extern CANMSG_DASErrorsTypeDef        CANMSG_DASErrors;
extern CANMSG_SteerStatusTypeDef      CANMSG_SteerStatus;
extern CANMSG_CarStatusTypeDef        CANMSG_CarStatus;
extern CANMSG_SetCarStatusTypeDef     CANMSG_SetCarStatus;
extern CANMSG_SpeedTypeDef            CANMSG_Speed;
extern CANMSG_HVVoltageTypeDef        CANMSG_HVVoltage;
extern CANMSG_HVCurrentTypeDef        CANMSG_HVCurrent;
extern CANMSG_HVTemperatureTypeDef    CANMSG_HVTemperature;
extern CANMSG_HVErrorsTypeDef         CANMSG_HVErrors;
extern CANMSG_HVFeedbacksTypeDef      CANMSG_HVFeedbacks;
extern CANMSG_TSStatusTypeDef         CANMSG_TSStatus;
extern CANMSG_SetTSStatusTypeDef      CANMSG_SetTSStatus;
extern CANMSG_LVCurrentTypeDef        CANMSG_LVCurrent;
extern CANMSG_LVVoltageTypeDef        CANMSG_LVVoltage;
extern CANMSG_LVTemperatureTypeDef    CANMSG_LVTemperature;
extern CANMSG_InvConnStatusTypeDef    CANMSG_InvConnStatus;
extern CANMSG_SetInvConnStatusTypeDef CANMSG_SetInvConnStatus;
extern CANMSG_SetPedRangeTypeDef      CANMSG_SetPedRange;
extern CANMSG_SetSteerRangeTypeDef    CANMSG_SetSteerRange;

// extern CANMSG_Inv_SetTorqueTypeDef    CANMSG_InvL_SetTorque;
// extern CANMSG_Inv_SetTorqueTypeDef    CANMSG_InvR_SetTorque;
// extern CANMSG_Inv_ResponseTypeDef     CANMSG_InvL_Status, CANMSG_InvL_IOInfo, CANMSG_InvL_Errors, CANMSG_InvL_Speed, CANMSG_InvL_MTemp, CANMSG_InvL_ITemp;
// extern CANMSG_Inv_ResponseTypeDef     CANMSG_InvR_Status, CANMSG_InvR_IOInfo, CANMSG_InvR_Errors, CANMSG_InvR_Speed, CANMSG_InvR_MTemp, CANMSG_InvR_ITemp;

extern CANMSG_PedValsTypeDef          CANMSG_PedVals;
extern CANMSG_CtrlOutTypeDef          CANMSG_CtrlOut;
extern CANMSG_SteerValTypeDef         CANMSG_SteerVal;
extern CANMSG_IMUAccTypeDef           CANMSG_IMUAcc;
extern CANMSG_IMUAngTypeDef           CANMSG_IMUAng;


/**
 * @brief Deserialize and save a new received message
 */
void CANMSG_process_RX(CAN_MessageTypeDef msg);

/**
 * @brief Send all new/available messages
 */
void CANMSG_flush_TX();


CANMSG_MetadataTypeDef* CANMSG_get_metadata_from_id(CAN_IdTypeDef id);

#endif