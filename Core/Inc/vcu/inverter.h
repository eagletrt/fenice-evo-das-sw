/*!
 * \file inverter.h
 * \date 2026-06-26
 * \authors Alessandro Bridi [ale.bridi15@gmail.com]
 * \ingroup Core
 *
 * \brief Generic inverter layer over a concrete inverter driver.
 *
 * \details Application-facing layer: it exposes plain commands (arm, set speed,
 *     set torque, ...) and owns the CAN plumbing (periodic setpoint TX +
 *     received-frame dispatch) on the single inverter network. It holds the
 *     concrete driver by value and calls its API directly - swapping inverter
 *     means changing the held handler type and the ephorus_api_* calls in
 *     inverter-api.c (just like leds.h holds and drives ws2812b). No vtables,
 *     no void pointers. Wheel addressing and telemetry are the driver's own
 *     types, since those are genuinely inverter-specific.
 */

#ifndef INVERTER_H
#define INVERTER_H

#include <stdint.h>

#include "ephorus.h"

#define INVERTER_DEFAULT_SPEED_RPM (2000) /*!< Default speed for the inverter in RPM. */

/*! \brief The single CAN network every inverter lives on. */
#define INVERTER_NETWORK CAN_COMMUNICATION_NETWORK_INVERTER

/*!
 * \brief Return codes for the Inverter module.
 */
enum InverterReturnCode {
    INVERTER_RC_OK,            /*!< Operation successful. */
    INVERTER_RC_NULL_POINTER,  /*!< A null pointer was passed to a function. */
    INVERTER_RC_INVALID_WHEEL, /*!< Wheel index out of range. */
    INVERTER_RC_TX_ERROR,      /*!< Queueing a setpoint frame for transmission failed. */
};

/*!
 * \brief File-static state of the Inverter module: the driver plus TX timing.
 */
struct InverterHandler {
    struct EphorusHandler driver; /*!< Concrete inverter driver (held by value, typed). */
    uint32_t last_tx_tick;        /*!< Tick of the last setpoint broadcast. */
};

#endif // INVERTER_H
