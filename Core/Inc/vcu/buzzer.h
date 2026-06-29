#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

// R2D defines
#define BUZZER_RD2_SOUND_DURATION_MS (3000) // Sound duration in ms for the RD2

/*!
 * \brief Return codes for the buzzer module APIs.
 */
enum BuzzerReturnCode {
    BUZZER_RC_OK,     /*!< Operation completed successfully */
    BUZZER_RC_ERROR,  /*!< Invalid parameters or uninitialized hardware pointers */
    BUZZER_RC_PLAYING /*!< The buzzer is currently active in async mode */
};

/*! 
 * \brief Signature for enabling the buzzer with specific characteristics.
 * \param frequency The target frequency in Hz.
 * \param amplitude The amplitude (volume) value between 0 and 1.
 * \note Regardless of the peripheral used, the two parameters have to be defined in the
 * callback signature but in the function implementation these can be ignored (e.g. GPIO).
 */
typedef enum BuzzerReturnCode (*buzzer_on_callback)(uint32_t frequency, float amplitude);

/*!
 * \brief Signature for disabling the buzzer. 
 * \note No parameters needed for the 'Off' callback.
 */
typedef enum BuzzerReturnCode (*buzzer_off_callback)(void);

/*!
 * \brief Signature for playing the buzzer in blocking way.
 * \param frequency The target frequency in Hz.
 * \param amplitude The amplitude (volume) value between 0 and 1.
 * \param duration_ms Time to stall execution in milliseconds.
 * \note Regardless of the peripheral used, the two parameters have to be defined in the
 * callback signature but in the function implementation these can be ignored (e.g. GPIO).
 */
typedef enum BuzzerReturnCode (*buzzer_delay_callback)(uint32_t frequency, float amplitude, uint32_t duration_ms);

/*!
 * \brief Signature for fetching system uptime (e.g., HAL_GetTick).
 * \return Current system time.
 */
typedef uint32_t (*buzzer_tick_callback)(void);

/*!
 * \brief Internal state and hardware interface for the buzzer.
 * It stores the function pointers used to bridge the logic to the physical hardware.
 */
struct BuzzerHandler {
    buzzer_on_callback buzzer_on;           /*!< Callback to enable the buzzer pin */
    buzzer_off_callback buzzer_off;         /*!< Callback to disable the buzzer pin */
    buzzer_delay_callback buzzer_play_sync; /*!< Callback for synchronous blocking playing */
    buzzer_tick_callback buzzer_get_tick;   /*!< Callback to retrieve elapsed time for async logic */

    uint32_t frequency;  /*!< Desired buzzer frequency in Hz */
    float amplitude;     /*!< Desired volume as a percentage (0-1) */
    uint32_t duration;   /*!< Target sound duration in milliseconds */
    uint32_t start_time; /*!< Timestamp (ms) when async play started */
    bool is_playing;     /*!< Flag indicating if buzzer is physically active */
};

#endif
