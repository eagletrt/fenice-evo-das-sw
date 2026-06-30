#ifndef PEDALS_H
#define PEDALS_H

#include <stdint.h>

#define PEDALS_MAX_BRAKE_PRESSURE (100.0f)
#define PEDALS_MAX_TORQUE_NM (30.0f)
#define PEDALS_BRAKE_THRESHOLD_LIGHT_BAR (1.2f)
#define PEDALS_COMMUNICATION_TIMEOUT_MS (100U) // TODO: figure out a real number

typedef void (*pedals_set_brake_light_callback)(bool brake_pressed);
typedef uint32_t (*pedals_get_tick_callback)(void);

enum PedalsReturnCode {
    PEDALS_RC_OK,
    PEDALS_RC_ERROR
};

struct PedalsHandler {
    float throttle;
    float brake;
    float brake_pressure;

    uint32_t last_tick;

    pedals_set_brake_light_callback set_brake_light;
    pedals_get_tick_callback get_tick;
};

#endif // PEDALS_H
