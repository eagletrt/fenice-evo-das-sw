#ifndef __REGEN_CONTROL_H
#define __REGEN_CONTROL_H

#include "encoders.h"

#define REG_CONTROL_RATE 100 // 100 Hz
#define REG_FILTER_SAMPLES 20
#define REG_FILTERED_BUFFER_SIZE 100
#define REG_SPEED_DT_MS (ENC_SPEED_PERIOD_MS)

#define REG_MIN_SPEED -0.5
#define REG_SPEED_THRESHOLD 0.5
#define REG_ACC_THRESHOLD 0.035

typedef struct regen_data_t
{
    double speed[REG_FILTER_SAMPLES];                // raw speed signal
    double speed_filtered[REG_FILTERED_BUFFER_SIZE]; // filtered speed signal
    double speed_filtered_diff;                      // difference between current and previous filtered speed signal

    double previous_torque;
    double acceleration_condition_should_regenerate;

    double speed_ref;
    double torque_ref;
} regen_data_t;

extern regen_data_t regen_data;

void regen_control_init(regen_data_t *reg);
void regen_add_speed_sample_rads(regen_data_t *reg, double speed_rads);
void regen_add_speed_sample(regen_data_t *reg, double speed_ms);
void regen_set_speed_ref(regen_data_t *reg, double speed_ref);
void regen_set_torque_ref(regen_data_t *reg, double torque_ref);
double regen_get_command(regen_data_t *reg);

#endif // __REGEN_CONTROL_H