#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/*!
 * \brief Names for checked shutdowns of DAS FRONT
 */
enum EncoderName : uint8_t {
    ENCODER_NAME_STEERING, /*!< Encoder steering wheel */
    ENCODER_NAME_COUNT     /*!< Total number of encoders */
};

/*!
 * \brief Possible return codes for encoder initialization function
 */
enum EncoderReturnCode : uint8_t {
    ENCODER_RC_OK,   /*!< Initialization successful */
    ENCODER_RC_ERROR /*!< Error during initialization */
};

/*!
 * \brief Struct that handles all relevant encoder information
 */
struct EncoderHandler {
    float steering_wheel_angle[ENCODER_NAME_COUNT]; /*!< Encoder angle */
};

#endif // ENCODER_H
