#ifndef __REGEN_CONTROL_H
#define __REGEN_CONTROL_H

#include "encoders.h"
#include "inverters.h"

#define REG_CONTROL_RATE 100 // 100 Hz
#define REG_FILTER_COEFFICIENT 0.1
#define REG_SPEED_DT_MS (ENC_SPEED_PERIOD_MS)

#define REG_MIN_SPEED 0.0
#define REG_SPEED_THRESHOLD 0.5
#define REG_ACC_THRESHOLD 0.035

typedef struct regen_data_t
{
    double speed;                           // raw speed signal
    double speed_prev;                      // previous raw speed signal
    double speed_diff;                      // difference between current and previous filtered speed signal

    double previous_torque;
    double acceleration_condition_should_regenerate;

    double speed_ref;
    double torque_ref;

    double torque_cutoff;
} regen_data_t;

extern regen_data_t regen_data;

void regen_control_init(regen_data_t *reg);
void regen_add_speed_sample_rads(regen_data_t *reg, double speed_rads);
void regen_add_speed_sample(regen_data_t *reg, double speed_ms);
void regen_set_speed_ref(regen_data_t *reg, double speed_ref);
void regen_set_torque_ref(regen_data_t *reg, double torque_ref);
double regen_get_command(regen_data_t *reg);

#endif // __REGEN_CONTROL_H