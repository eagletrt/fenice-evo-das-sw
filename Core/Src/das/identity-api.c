#include "identity-api.h"
#include "can-primary-api.h"
#include "can-version.h"
#include "can-communications-api.h"
#include "eagletrt-api.h"
#include <time.h>

EAGLETRT_STATIC struct IdentityHandler identity_handler;

enum IdentityReturnCode identity_api_init(void) {
    struct tm timeinfo = { 0 };
    strptime(__DATE__ " " __TIME__, "%b %d %Y %H:%M:%S", &timeinfo);
    identity_handler = (struct IdentityHandler){
        .build_time = mktime(&timeinfo),
        .last_tick_ms_status = 0,
        .last_tick_ms_version = 0,
        .last_tick_ms_libcan_version = 0,
    };
    return IDENTITY_RC_OK;
}

enum IdentityReturnCode identity_api_send_state(enum CanPrimaryDasfrontfsmStatus status) {
    union CanPrimaryMessages message;
    message.dasfrontfsm = (struct CanPrimaryDasfrontfsm){
        .status = status,
    };
    struct CanCommunicationFrame frame;
    if (can_primary_api_serialize_from_id(CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTFSM, &message, frame.data) != -1) {
        frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTFSM;
        frame.length = can_primary_byte_size_dasfrontfsm;
        EAGLETRT_API_UNUSED(can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame));
    }
    return IDENTITY_RC_OK;
}

enum IdentityReturnCode identity_api_periodically_send_state(enum CanPrimaryDasfrontfsmStatus status, uint32_t tick_ms) {
    if (tick_ms - identity_handler.last_tick_ms_status >= can_primary_cycle_time_dasfrontfsm) {
        identity_handler.last_tick_ms_status = tick_ms;

        EAGLETRT_API_UNUSED(identity_api_send_state(status));
    }
    return IDENTITY_RC_OK;
}

enum IdentityReturnCode identity_api_periodically_send_version(uint32_t tick_ms) {
    if (tick_ms - identity_handler.last_tick_ms_version >= can_primary_cycle_time_dasfrontversion) {
        identity_handler.last_tick_ms_version = tick_ms;

        // version
        union CanPrimaryMessages message;
        message.dasfrontversion = (struct CanPrimaryDasfrontversion){
            .major = IDENTITY_VERSION_MAJOR,
            .minor = IDENTITY_VERSION_MINOR,
            .patch = IDENTITY_VERSION_PATCH,
        };
        struct CanCommunicationFrame frame;
        if (can_primary_api_serialize_from_id(CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTVERSION, &message, frame.data) != -1) {
            frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTVERSION;
            frame.length = can_primary_byte_size_dasfrontversion;
            EAGLETRT_API_UNUSED(can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame));
        }

        message.dasfrontversioninfo = (struct CanPrimaryDasfrontversioninfo){
            .buildtime = identity_handler.build_time,
            .commithash = IDENTITY_VERSION_INFO_COMMIT_HASH,
            .dirty = IDENTITY_VERSION_DIRTY,
        };
        if (can_primary_api_serialize_from_id(CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTVERSIONINFO, &message, frame.data) != -1) {
            frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTVERSIONINFO;
            frame.length = can_primary_byte_size_dasfrontversioninfo;
            EAGLETRT_API_UNUSED(can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame));
        }
    }
    return IDENTITY_RC_OK;
}

enum IdentityReturnCode identity_api_periodically_send_libcan_version(uint32_t tick_ms) {
    if (tick_ms - identity_handler.last_tick_ms_libcan_version >= can_primary_cycle_time_dasfrontlibcanversion) {
        identity_handler.last_tick_ms_libcan_version = tick_ms;

        // libcan version
        union CanPrimaryMessages message;
        message.dasfrontlibcanversion = (struct CanPrimaryDasfrontlibcanversion){
            .major = can_version_major,
            .minor = can_version_minor,
            .patch = can_version_patch,
        };
        struct CanCommunicationFrame frame;
        if (can_primary_api_serialize_from_id(CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTLIBCANVERSION, &message, frame.data) != -1) {
            frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTLIBCANVERSION;
            frame.length = can_primary_byte_size_dasfrontlibcanversion;
            EAGLETRT_API_UNUSED(can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame));
        }

        message.dasfrontlibcanversioninfo = (struct CanPrimaryDasfrontlibcanversioninfo){
            .commithash = 0,
            .dirty = 0,
            .generationtime = can_generation_time,
        };
        if (can_primary_api_serialize_from_id(CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTLIBCANVERSIONINFO, &message, frame.data) != -1) {
            frame.id = CAN_PRIMARY_MESSAGE_FRAME_ID_DASFRONTLIBCANVERSIONINFO;
            frame.length = can_primary_byte_size_dasfrontlibcanversioninfo;
            EAGLETRT_API_UNUSED(can_communications_api_add_to_tx_buffer(CAN_COMMUNICATION_NETWORK_PRIMARY, &frame));
        }
    }
    return IDENTITY_RC_OK;
}
