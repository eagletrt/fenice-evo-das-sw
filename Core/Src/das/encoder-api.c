#include "can-communications-api.h"
#include "can-primary-api.h"
#include "encoder-api.h"
#include "eagletrt.h"
#include <string.h>

EAGLETRT_STATIC struct EncoderHandler encoder_api_handler;

enum EncoderReturnCode encoder_api_init(void) {
    // Initialize encoder value to 0 to indicate no data
    memset(&encoder_api_handler, 0, sizeof(encoder_api_handler));

    return ENCODER_RC_OK;
}

float encoder_api_get_angle(enum EncoderName encoder) {
    return encoder_api_handler.steering_wheel_angle[encoder];
}

enum EncoderReturnCode encoder_api_set_angle(enum EncoderName encoder, float angle) {
    encoder_api_handler.steering_wheel_angle[encoder] = angle;

    return ENCODER_RC_OK;
}

enum EncoderReturnCode encoder_api_periodically_send_angle(uint32_t tick) {
    EAGLETRT_STATIC uint32_t last_tick = 0;
    if (tick - last_tick >= can_primary_cycle_time_steeringencoder) {
        last_tick = tick;
        union CanPrimaryMessages message;

        message.steeringencoder = (struct CanPrimarySteeringencoder){
            .angle = encoder_api_handler.steering_wheel_angle[ENCODER_NAME_STEERING],
        };

        struct CanCommunicationFrame frame = {
            .id = CAN_PRIMARY_MESSAGE_FRAME_ID_STEERINGENCODER,
            .length = can_primary_byte_size_steeringencoder,
        };
        if (can_primary_api_serialize_from_id(frame.id, &message, frame.data) < 0) {
            return ENCODER_RC_ERROR;
        }

        if (can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame) != CAN_COMMUNICATION_RC_OK) {
            return ENCODER_RC_ERROR;
        }
    }
    return ENCODER_RC_OK;
}
