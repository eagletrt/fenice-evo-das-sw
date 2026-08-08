#include "logger-api.h"
#include "pal-api.h"
#include "eagletrt.h"
#include "arena-allocator-api.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define LOGGER_MAX_LINE_SIZE (128U)

/*!
 * \brief Internal module handler.
 * \details Hidden from external linkage to enforce API-only access.
 */
EAGLETRT_STATIC struct LoggerHandler logger_handler;

enum LoggerReturnCode logger_api_init(struct LoggerConfig config, bool logger_state) {
    if (config.send == NULL) {
        return LOGGER_RC_NULL_POINTER;
    }

    arena_allocator_api_init(&logger_handler.arena_handler);
    pal_api_init(&logger_handler.pal_handler, 1, 128, 128, NULL, config.send, config.cs_enter, config.cs_exit, &logger_handler.arena_handler);

    logger_handler.logger_state = logger_state;

    return LOGGER_RC_OK;
}

void logger_api_set_state(bool logger_state) {
    logger_handler.logger_state = logger_state;
}

enum LoggerReturnCode logger_api_log(enum LoggerLevel level, const char *format, ...) {
    if (logger_handler.pal_handler.send == NULL || format == NULL) {
        return LOGGER_RC_NULL_POINTER;
    }

    // Return OK immediately if the logger is currently muted/disabled
    if (!logger_handler.logger_state) {
        return LOGGER_RC_OK;
    }

    char final_buffer[LOGGER_MAX_LINE_SIZE];

    // Lookup table replacing the switch-case matrix
    char *const log_headers[] = {
        [LOGGER_LEVEL_DEBUG] = "[DEBUG]",
        [LOGGER_LEVEL_INFO] = "[INFO]",
        [LOGGER_LEVEL_WARN] = "[WARN]",
        [LOGGER_LEVEL_ERROR] = "[ERROR]"
    };

    // Safely check bounds using LOGGER_LEVEL_COUNT before accessing memory
    // For default [LOG] will be left in case the logger level specified is not valid
    const char *header = (level < LOGGER_LEVEL_COUNT) ? log_headers[level] : "[LOG]";

    int offset = snprintf(final_buffer, sizeof(final_buffer), "%s ", header);

    // Verify no anomalies or truncations occurred during tag placement
    if (offset < 0 || offset >= (int)sizeof(final_buffer)) {
        return LOGGER_RC_TRANSMISSION_ERROR;
    }

    // Process variable args into the remaining space of the local buffer
    va_list args;
    va_start(args, format);
    int body_len = vsnprintf(final_buffer + offset, sizeof(final_buffer) - (size_t)offset, format, args);
    va_end(args);

    if (body_len < 0) {
        return LOGGER_RC_TRANSMISSION_ERROR; // Format parsing exception
    }

    // Measure the actual string safely populated inside the buffer boundary
    size_t actual_len = strlen(final_buffer);

    if (actual_len < sizeof(final_buffer) - 3U) {
        final_buffer[actual_len++] = '\n'; // Append newline if space allows
        final_buffer[actual_len++] = '\r'; // Append carriage return if space allows
        final_buffer[actual_len] = '\0';   // Ensure null termination
    }

    // Determine transmission size including the string null terminator
    uint32_t tx_bytes = (uint32_t)(actual_len + 1U);

    // Queue the formatted record into PAL
    enum PalReturnCode pal_rc = pal_api_add_to_tx_queue(&logger_handler.pal_handler, final_buffer, tx_bytes);
    if (pal_rc != PAL_RC_OK) {
        if (pal_rc == PAL_RC_QUEUE_FULL) {
            return LOGGER_RC_BUFFER_FULL;
        }
        return LOGGER_RC_TRANSMISSION_ERROR;
    }

    // Process/Flush the queue immediately so diagnostics output synchronously
    pal_rc = pal_api_process_tx(&logger_handler.pal_handler);
    if (pal_rc != PAL_RC_OK) {
        if (pal_rc == PAL_RC_QUEUE_FULL) {
            return LOGGER_RC_BUFFER_FULL;
        }
        return LOGGER_RC_TRANSMISSION_ERROR;
    }

    return LOGGER_RC_OK;
}
