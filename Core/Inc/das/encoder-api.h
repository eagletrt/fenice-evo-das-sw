#include "encoder.h"

#include <stdint.h>

/*!
 * \brief Initializes the module
 *
 * \retval ENCODER_RC_OK if successful
 * \retval ENCODER_RC_ERROR if fail
 */
enum EncoderReturnCode encoder_api_init(void);

/*!
 * \brief Get the current angle of a encoder steering wheel
 *
 * \param encoder The encoder to query
 * 
 * \retval float angle of \p encoder steering wheel
 */
float encoder_api_get_angle(enum EncoderName encoder);

/*!
 * \brief  Set the current state of a encoder line
 *
 * \param encoder The encoder to be set
 * \param state The state to set \p encoder to
 *
 * \retval ENCODER_RC_OK if successful
 * \retval ENCODER_RC_ERROR if \p encoder or \p state are out of bounds
 */
enum EncoderReturnCode encoder_api_set_angle(enum EncoderName encoder, float angle);

/*!
 * \brief Periodically send the encoder angle over CAN
 *
 * \param tick The current tick count
 *
 * \retval ENCODER_RC_OK if successful
 * \retval ENCODER_RC_ERROR if fail
 */
enum EncoderReturnCode encoder_api_periodically_send_angle(uint32_t tick);
