#ifndef BUZZER_API_H
#define BUZZER_API_H

#include "buzzer.h"

/*!
 * \brief Bulk initializes the entire buzzer subsystem and links logic to hardware.
 * This function performs an "all-or-nothing" initialization.
 *
 * \note Re-calling this function will reset all internal states (duration, 
 * frequency, etc.) to zero for all buzzers.
 *
 * \param[in] on_callback Callback to enable the buzzer pin.
 * \param[in] off_callback Callback to disable the buzzer pin.
 * \param[in] play_sync_callback Callback for synchronous blocking playing.
 * \param[in] get_tick_callback Callback to retrieve elapsed time for async logic.
 *
 * \retval BUZZER_RC_OK if the buzzer subsystem was initialized successfully.
 * \retval BUZZER_RC_ERROR if the buzzer subsystem could not be initialized.
 */
enum BuzzerReturnCode buzzer_api_init(
    buzzer_on_callback on_callback,
    buzzer_off_callback off_callback,
    buzzer_delay_callback play_sync_callback,
    buzzer_tick_callback get_tick_callback);

/*!
 * \brief Plays the buzzer and blocks the CPU until duration elapses.
 * \warning No other code will run until the buzzer stops.
 *
 * \retval BUZZER_RC_OK if the buzzer played in sync correctly.
 * \retval BUZZER_RC_ERROR if it was not possible to play the buzzer.
 */
enum BuzzerReturnCode buzzer_api_play_sync(void);

/*!
 * \brief Starts the buzzer is not already playing and monitors the playing duration
 * in a non-blocking way.
 * \note Should be called in the main loop. If the duration has passed,
 * it automatically calls the buzzer off callback.
 *
 * \retval BUZZER_RC_OK if the buzzer just finished playing.
 * \retval BUZZER_RC_PLAYING if the buzzer is currently playing.
 * \retval BUZZER_RC_ERROR if it was not possible to update the state or it was not possible
 * to start/stop the buzzer.
 */
enum BuzzerReturnCode buzzer_api_play_async(void);

/*!
 * \brief Routine to be called in the main loop to handle buzzer state and timing.
 * It checks if the buzzer is playing and turns it off if the duration has elapsed.
 */
void buzzer_api_routine(void);

/*!
 * \brief Forces the buzzer OFF and resets the buzzer state.
 *
 * \retval BUZZER_RC_OK if the handler has been reset and the buzzer stopped.
 * \retval BUZZER_RC_ERROR if a reset could not be achieved or \c buzzer_type is unknown.
 */
enum BuzzerReturnCode buzzer_api_reset(void);

/*!
 * \brief Updates the duration for the next play command.
 *
 * \param[in] duration Time in milliseconds.
 * \retval BUZZER_RC_OK if \c duration has been changed.
 * \retval BUZZER_RC_ERROR if \c buzzer_type is unknownw
 */
enum BuzzerReturnCode buzzer_api_set_duration(uint32_t duration);

/*!
 * \brief Sets the buzzer frequency.
 *
 * \param[in] frequency Frequency in Hz.
 * \retval BUZZER_RC_OK if \c frequency has been changed.
 * \retval BUZZER_RC_ERROR if \c buzzer_type is unknownw
 */
enum BuzzerReturnCode buzzer_api_set_frequency(uint32_t frequency);

/*!
 * \brief Sets the buzzer amplitude.
 *
 * \param[in] amplitude Amplitude as a percentage (0-1).
 * \retval BUZZER_RC_OK if \c amplitude has been changed.
 * \retval BUZZER_RC_ERROR if \c buzzer_type is unknownw or the \c amplitude value was out of range.
 */
enum BuzzerReturnCode buzzer_api_set_amplitude(float amplitude);

/*!
 * \brief Retrieves the current configured duration.
 *
 * \return The play duration in milliseconds. 
 * \note Returns 0 if \c buzzer_type is invalid.
 */
uint32_t buzzer_api_get_duration(void);

/*!
 * \brief Gets the currently set buzzer frequency.
 *
 * \return The frequency in Hz. 
 * \note Returns 0 if \c buzzer_type is invalid.
 */
uint32_t buzzer_api_get_frequency(void);

/*!
 * \brief Gets the currently set buzzer amplitude.
 *
 * \return The amplitude percentage between 0.0 and 1.0. 
 * \note Returns 0 if \c buzzer_type is invalid.
 */
float buzzer_api_get_amplitude(void);

/*!
 * \brief Check if the buzzer is playing a sound.
 *
 * \retval \c true The buzzer is currently active/playing.
 * \retval \c false The buzzer is idle, or the \c buzzer_type is invalid.
 * 
 */
bool buzzer_api_is_playing(void);

#endif
