#include <string.h>
#include "buzzer-api.h"
#include "eagletrt.h"

EAGLETRT_STATIC struct BuzzerHandler buzzer_handler;

enum BuzzerReturnCode buzzer_api_init(
    buzzer_on_callback on_callback,
    buzzer_off_callback off_callback,
    buzzer_delay_callback play_sync_callback,
    buzzer_tick_callback get_tick_callback) {

    if (on_callback == NULL || off_callback == NULL ||
        play_sync_callback == NULL || get_tick_callback == NULL) {
        return BUZZER_RC_ERROR;
    }

    buzzer_handler = (struct BuzzerHandler){
        .buzzer_on = on_callback,
        .buzzer_off = off_callback,
        .buzzer_play_sync = play_sync_callback,
        .buzzer_get_tick = get_tick_callback
    };

    if (buzzer_handler.buzzer_off() != BUZZER_RC_OK) {
        return BUZZER_RC_ERROR;
    }

    return BUZZER_RC_OK;
}

enum BuzzerReturnCode buzzer_api_play_sync(void) {
    if (buzzer_handler.buzzer_play_sync == NULL) {
        return BUZZER_RC_ERROR;
    }

    return buzzer_handler.buzzer_play_sync(buzzer_handler.frequency, buzzer_handler.amplitude, buzzer_handler.duration);
}

enum BuzzerReturnCode buzzer_api_play_async(void) {
    if (buzzer_handler.buzzer_on == NULL || buzzer_handler.buzzer_off == NULL || buzzer_handler.buzzer_get_tick == NULL) {
        return BUZZER_RC_ERROR;
    }

    // check elapsed time
    uint32_t current_time = buzzer_handler.buzzer_get_tick();

    if (!buzzer_handler.is_playing) {
        // if NOT playing, start the buzzer
        if (buzzer_handler.buzzer_on(buzzer_handler.frequency, buzzer_handler.amplitude) == BUZZER_RC_ERROR) {
            return BUZZER_RC_ERROR;
        }

        buzzer_handler.start_time = current_time;
        buzzer_handler.is_playing = true;
        return BUZZER_RC_PLAYING;
    } else if ((current_time - buzzer_handler.start_time) >= buzzer_handler.duration) {
        // buzzer is currently playing
        if (buzzer_handler.buzzer_off() != BUZZER_RC_OK) {
            return BUZZER_RC_ERROR;
        }

        buzzer_handler.is_playing = false;
        return BUZZER_RC_OK;
    }

    return BUZZER_RC_PLAYING;
}

void buzzer_api_routine(void) {
    if (buzzer_handler.buzzer_off == NULL || buzzer_handler.buzzer_get_tick == NULL) {
        return;
    }

    if (buzzer_handler.is_playing) {
        uint32_t current_time = buzzer_handler.buzzer_get_tick();
        if ((current_time - buzzer_handler.start_time) >= buzzer_handler.duration) {
            buzzer_handler.buzzer_off();
            buzzer_handler.is_playing = false;
        }
    }
}

enum BuzzerReturnCode buzzer_api_reset(void) {
    if (buzzer_handler.buzzer_off != NULL) {
        if (buzzer_handler.buzzer_off() != BUZZER_RC_OK)
            return BUZZER_RC_ERROR;
    }

    buzzer_handler.is_playing = false;
    buzzer_handler.duration = 0;
    buzzer_handler.frequency = 0;
    buzzer_handler.amplitude = 0;

    return BUZZER_RC_OK;
}

enum BuzzerReturnCode buzzer_api_set_duration(uint32_t duration) {
    buzzer_handler.duration = duration;
    return BUZZER_RC_OK;
}

enum BuzzerReturnCode buzzer_api_set_frequency(uint32_t frequency) {
    buzzer_handler.frequency = frequency;
    return BUZZER_RC_OK;
}

enum BuzzerReturnCode buzzer_api_set_amplitude(float amplitude) {
    if (amplitude >= 0.0f && amplitude <= 1.0f) {
        buzzer_handler.amplitude = amplitude;
        return BUZZER_RC_OK;
    }
    return BUZZER_RC_ERROR;
}

uint32_t buzzer_api_get_duration(void) {
    return buzzer_handler.duration;
}

uint32_t buzzer_api_get_frequency() {
    return buzzer_handler.frequency;
}

float buzzer_api_get_amplitude(void) {
    return buzzer_handler.amplitude;
}

bool buzzer_api_is_playing(void) {
    return buzzer_handler.is_playing;
}
