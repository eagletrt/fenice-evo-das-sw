#include "tsac-api.h"
#include "can-primary-api.h"
#include "can-primary.h"
#include "eagletrt.h"
#include "can-communications-api.h"
#include <string.h>

EAGLETRT_STATIC struct TsacHandler tsac_handler;

void tsac_api_init(void) {
    memset(&tsac_handler, 0, sizeof(tsac_handler));
}

enum TsacStatus tsac_api_get_status(void) {
    return tsac_handler.status;
}

enum TsacReturnCode tsac_api_set_status(enum TsacStatus status) {
    if (status >= TSAC_STATUS_COUNT) {
        return TSAC_RC_INVALID_STATUS;
    }
    tsac_handler.status = status;
    return TSAC_RC_OK;
}

enum TsacReturnCode tsac_api_ask_power_on(void) {
    union CanPrimaryMessages msg = { 0 };
    msg.ecu_set_hv_bms_status.targetstatus = true;
    struct CanCommunicationFrame frame = { 0 };
    frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_ECU_SET_HV_BMS_STATUS;
    frame.length = can_primary_byte_size_ecu_set_hv_bms_status;
    if (can_primary_api_serialize_from_id(frame.id, &msg, frame.data) != -1) {
        if (can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame) != CAN_COMMUNICATION_RC_OK) {
            return TSAC_RC_INVALID_STATUS;
        }
    } else {
        return TSAC_RC_INVALID_STATUS;
    }
    return TSAC_RC_OK;
}

enum TsacReturnCode tsac_api_ask_power_off(void) {
    union CanPrimaryMessages msg = { 0 };
    msg.ecu_set_hv_bms_status.targetstatus = false;
    struct CanCommunicationFrame frame = { 0 };
    frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_ECU_SET_HV_BMS_STATUS;
    frame.length = can_primary_byte_size_ecu_set_hv_bms_status;
    if (can_primary_api_serialize_from_id(frame.id, &msg, frame.data) != -1) {
        if (can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame) != CAN_COMMUNICATION_RC_OK) {
            return TSAC_RC_INVALID_STATUS;
        }
    } else {
        return TSAC_RC_INVALID_STATUS;
    }
    return TSAC_RC_OK;
}

enum TsacStatus tsac_api_convert_from_can_status(enum CanPrimaryHvBmsStatusName can_status) {
    switch (can_status) {
        case CAN_PRIMARY_HV_BMS_STATUS_NAME_FATAL:
            return TSAC_STATUS_FATAL;
        case CAN_PRIMARY_HV_BMS_STATUS_NAME_WAIT_AIRN_CLOSE:
        case CAN_PRIMARY_HV_BMS_STATUS_NAME_WAIT_AIRP_CLOSE:
        case CAN_PRIMARY_HV_BMS_STATUS_NAME_WAIT_PRECHARGE:
            return TSAC_STATUS_PRECHARGING;
        case CAN_PRIMARY_HV_BMS_STATUS_NAME_TS_ON:
            return TSAC_STATUS_ON;
        case CAN_PRIMARY_HV_BMS_STATUS_NAME_INIT:
        case CAN_PRIMARY_HV_BMS_STATUS_NAME_IDLE:
            return TSAC_STATUS_FATAL;
        default:
            return TSAC_STATUS_UNKNOWN;
    }
}
