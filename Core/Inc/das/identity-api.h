#ifndef IDENTITY_API_H
#define IDENTITY_API_H

#include "identity.h"
#include "can-primary.h"
#include <stdint.h>

/*!
 * \brief Initialize the identity API.
 *
 * \retval IDENTITY_RC_OK if the identity API was initialized successfully.
 * \retval IDENTITY_RC_ERROR if there was an error initializing the identity API.
 */
enum IdentityReturnCode identity_api_init(void);

/*!
 * \brief Send state identity informations.
 *
 * \param status The current status of the primary DAS front FSM.
 *
 * \retval IDENTITY_RC_OK if the identity information was sent successfully.
 * \retval IDENTITY_RC_ERROR if there was an error sending the identity information.
 */
enum IdentityReturnCode identity_api_send_state(enum CanPrimaryDasfrontfsmStatus status);

/*!
 * \brief Periodically send state identity informations.
 *
 * \param status The current status of the primary DAS front FSM.
 * \param tick_ms The tick interval in milliseconds.
 *
 * \retval IDENTITY_RC_OK if the identity information was sent successfully.
 * \retval IDENTITY_RC_ERROR if there was an error sending the identity information.
 */
enum IdentityReturnCode identity_api_periodically_send_state(enum CanPrimaryDasfrontfsmStatus status, uint32_t tick_ms);

/*!
 * \brief Periodically send version identity informations.
 *
 * \param tick_ms The tick interval in milliseconds.
 *
 * \retval IDENTITY_RC_OK if the identity information was sent successfully.
 * \retval IDENTITY_RC_ERROR if there was an error sending the identity information.
 */
enum IdentityReturnCode identity_api_periodically_send_version(uint32_t tick_ms);

/*!
 * \brief Periodically send libcan version identity informations.
 *
 * \param tick_ms The tick interval in milliseconds.
 *
 * \retval IDENTITY_RC_OK if the identity information was sent successfully.
 * \retval IDENTITY_RC_ERROR if there was an error sending the identity information.
 */
enum IdentityReturnCode identity_api_periodically_send_libcan_version(uint32_t tick_ms);

#endif // IDENTITY_API_H
