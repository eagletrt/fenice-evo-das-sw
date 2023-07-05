#ifndef CANMSG_H
#define CANMSG_H

#include "can.h"
#include "stdbool.h"
#include "../Lib/can/lib/primary/primary_network.h"
#include "../Lib/can/lib/secondary/secondary_network.h"


typedef struct {
    uint32_t timestamp;      /*< Timestamp of when the message was received/sent */
    bool is_new;             /*< If the message data has yet to be processed     */
    CAN_HandleTypeDef *hcan; /*< CAN peripheral used to send/receive the message */
} CANMSG_MetadataTypeDef;


/* Primary Network */
typedef struct { CANMSG_MetadataTypeDef info; primary_das_version_converted_t data;      } CANMSG_DASVersionTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_das_errors_converted_t data;       } CANMSG_DASErrorsTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_steer_status_converted_t data;     } CANMSG_SteerStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_car_status_t data;       } CANMSG_CarStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_ecu_feedbacks_converted_t data;       } CANMSG_ECUFeedbacksTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_car_status_t data;   } CANMSG_SetCarStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_speed_converted_t data; } CANMSG_SpeedTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_voltage_t data;       } CANMSG_HVVoltageTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_current_t data;       } CANMSG_HVCurrentTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_temp_t data;          } CANMSG_HVTemperatureTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_errors_t data;        } CANMSG_HVErrorsTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_hv_feedbacks_status_t data; } CANMSG_HVFeedbacksTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_ts_status_t data;        } CANMSG_TSStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_ts_status_das_t data;    } CANMSG_SetTSStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_lv_currents_converted_t data;       } CANMSG_LVCurrentTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_lv_cells_voltage_t data;       } CANMSG_LVVoltageTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_lv_cells_temp_t data;   } CANMSG_LVTemperatureTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_inverter_connection_status_t data;     } CANMSG_InvConnStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_ptt_status_t data; } CANMSG_SetPTTStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_ptt_status_t data; } CANMSG_PTTStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_inverter_connection_status_t data; } CANMSG_SetInvConnStatusTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_set_pedal_calibration_t data; } CANMSG_SetPedalCalibrationTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_pedal_calibration_ack_t data; } CANMSG_PedalCalibrationAckTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_ambient_temperature_t data; } CANMSG_AmbientTemperatureTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; primary_control_output_converted_t data; } CANMSG_CtrlOutTypeDef;

/* Secondary Network */
typedef struct { CANMSG_MetadataTypeDef info; secondary_pedals_output_converted_t data;  } CANMSG_PedValsTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; secondary_steering_angle_converted_t data; } CANMSG_SteerValTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; secondary_imu_acceleration_converted_t data; } CANMSG_IMUAccTypeDef;
typedef struct { CANMSG_MetadataTypeDef info; secondary_imu_angular_rate_converted_t data; } CANMSG_IMUAngTypeDef;


extern CANMSG_DASVersionTypeDef       CANMSG_DASVersion;
extern CANMSG_DASErrorsTypeDef        CANMSG_DASErrors;
extern CANMSG_SteerStatusTypeDef      CANMSG_SteerStatus;
extern CANMSG_CarStatusTypeDef        CANMSG_CarStatus;
extern CANMSG_ECUFeedbacksTypeDef     CANMSG_EcuFeedbacks;
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
extern CANMSG_SetPTTStatusTypeDef     CANMSG_SetPTTStatus;
extern CANMSG_PTTStatusTypeDef        CANMSG_PTTStatus;
extern CANMSG_SetPedalCalibrationTypeDef CANMSG_SetPedalsCalibration;
extern CANMSG_PedalCalibrationAckTypeDef CANMSG_PedalsCalibrationAck;

extern CANMSG_PedValsTypeDef          CANMSG_PedVals;
extern CANMSG_CtrlOutTypeDef          CANMSG_CtrlOut;
extern CANMSG_SteerValTypeDef         CANMSG_SteerVal;
extern CANMSG_IMUAccTypeDef           CANMSG_IMUAcc;
extern CANMSG_IMUAngTypeDef           CANMSG_IMUAng;


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


CANMSG_MetadataTypeDef* CANMSG_get_primary_metadata_from_id(CAN_IdTypeDef id);
CANMSG_MetadataTypeDef* CANMSG_get_secondary_metadata_from_id(CAN_IdTypeDef id);

#endif