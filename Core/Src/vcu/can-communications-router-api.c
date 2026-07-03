#include "can-communications-router-api.h"
#include "can-communications.h"
#include "can-primary.h"
#include "inverter-api.h"
#include "can-primary-api.h"
#include "can-inverters-api.h"
#include "tsac-api.h"
#include "vehicle-api.h"
#include "pedals-api.h"
#include <stdio.h>

enum CanCommunicationReturnCode can_communications_router_api_receive_primary(const struct CanCommunicationFrame *frame) {
    if (frame == NULL) {
        return CAN_COMMUNICATION_RC_NULL_POINTER;
    }

    if (!can_primary_api_id_is_valid(frame->id)) {
        return CAN_COMMUNICATION_RC_INVALID_NETWORK;
    }

    union CanPrimaryMessages message = { 0 };
    if (can_primary_api_deserialize_from_id(frame->id, (uint8_t *)frame->data, &message) != 0) {
        return CAN_COMMUNICATION_RC_ERROR;
    }
    switch (frame->id) {
        case CAN_PRIMARY_MESSAGE_FRAME_ID_STEERING_WHEEL_SET_ECU_STATUS: {
            vehicle_api_request_state(message.steering_wheel_set_ecu_status.targetstatus);
            break;
        }
        case CAN_PRIMARY_MESSAGE_FRAME_ID_HV_BMS_FEEDBACK_STATUS: {
            vehicle_api_set_shutdown_end_closed(message.hv_bms_feedback_status.sdend == CAN_PRIMARY_HV_BMS_FEEDBACK_STATUS_SDEND_HIGH);
            vehicle_api_set_higher_than_60v(message.hv_bms_feedback_status.tsover60v);
            break;
        }
        case CAN_PRIMARY_MESSAGE_FRAME_ID_HV_BMS_STATUS: {
            tsac_api_set_status(tsac_api_convert_from_can_status(message.hv_bms_status.name));
            break;
        }
        case CAN_PRIMARY_MESSAGE_FRAME_ID_PEDALS_THROTTLE: {
            float travel_pct = ((message.pedals_throttle.status == CAN_PRIMARY_PEDALS_THROTTLE_STATUS_OK) || (message.pedals_throttle.status == CAN_PRIMARY_PEDALS_THROTTLE_STATUS_IMPLAUSIBILITY_RECOVERABLE)) ? message.pedals_throttle.travel_pct : 0.0f;
            pedals_api_set_throttle(travel_pct);
            break;
        }
        case CAN_PRIMARY_MESSAGE_FRAME_ID_PEDALS_BRAKE: {
            float travel_pct = message.pedals_brake.travel_pct;
            float brake_pressure = message.pedals_brake.pressurefront_bar;
            pedals_api_set_brake(travel_pct);
            pedals_api_set_brake_pressure(brake_pressure);
            break;
        }
    }

    return CAN_COMMUNICATION_RC_OK;
}

enum CanCommunicationReturnCode can_communications_router_api_receive_inverter(const struct CanCommunicationFrame *frame) {
    if (frame == NULL) {
        return CAN_COMMUNICATION_RC_NULL_POINTER;
    }

    if (!can_inverters_api_id_is_valid(frame->id)) {
        return CAN_COMMUNICATION_RC_INVALID_NETWORK;
    }

    return inverter_api_on_receive(frame);
}
