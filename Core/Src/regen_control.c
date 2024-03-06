#include "regen_control.h"

regen_data_t regen_data;

static int _thresholded_check(double value, double limit, double threshold)
{
    if (value < limit - threshold)
    {
        return -1;
    }
    else if (value > limit + threshold)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

double _median_filter(regen_data_t *reg)
{
    double sorted[REG_FILTER_SAMPLES];
    // bubble sort
    for (int i = 0; i < REG_FILTER_SAMPLES; i++)
    {
        sorted[i] = reg->speed[i];
    }
    for (int i = 0; i < REG_FILTER_SAMPLES; i++)
    {
        for (int j = 0; j < REG_FILTER_SAMPLES - i - 1; j++)
        {
            if (sorted[j] > sorted[j + 1])
            {
                double temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    return sorted[REG_FILTER_SAMPLES / 2];
}

void regen_control_init(regen_data_t *reg)
{
    reg->speed_filtered_diff = 0.0;
    reg->speed_ref = 0.0;
    reg->torque_ref = 0.0;
    reg->previous_torque = 0.0;
    reg->acceleration_condition_should_regenerate = 0;
}
void regen_add_speed_sample_rads(regen_data_t *reg, double speed_rads)
{
    regen_add_speed_sample(reg, speed_rads * ENC_WHEEL_RADIUS);
}
void regen_add_speed_sample(regen_data_t *reg, double speed)
{
    for (int i = 0; i < REG_FILTERED_BUFFER_SIZE - 1; i++)
    {
        reg->speed_filtered[i] = reg->speed_filtered[i + 1];
    }
    for (int i = 0; i < REG_FILTER_SAMPLES - 1; i++)
    {
        reg->speed[i] = reg->speed[i + 1];
    }
    reg->speed[REG_FILTER_SAMPLES - 1] = speed;

    reg->speed_filtered[REG_FILTERED_BUFFER_SIZE - 1] = _median_filter(reg);
    reg->speed_filtered_diff = reg->speed_filtered[REG_FILTERED_BUFFER_SIZE - 1] - reg->speed_filtered[0];
}
void regen_set_speed_ref(regen_data_t *reg, double speed_ref)
{
    reg->speed_ref = speed_ref;
}
void regen_set_torque_ref(regen_data_t *reg, double torque_ref)
{
    reg->torque_ref = torque_ref;
}
double _command_impl(regen_data_t *reg, double speed)
{
    switch (_thresholded_check(reg->speed_filtered_diff, -REG_ACC_THRESHOLD, REG_ACC_THRESHOLD))
    {
    case -1: // lower than threshold (decelerating)
        reg->acceleration_condition_should_regenerate = 1;
        break;
    case 0: // within threshold
        break;
    case 1: // higher than threshold (accelerating)
        reg->acceleration_condition_should_regenerate = 0;
        break;
    }
    if (speed < REG_MIN_SPEED || reg->acceleration_condition_should_regenerate == 0)
    {
        return 0.0;
    }

    switch (_thresholded_check(speed, reg->speed_ref, REG_SPEED_THRESHOLD))
    {
    case -1: // lower than threshold
        return reg->torque_ref;
    case 0: // within threshold
        return reg->previous_torque;
    case 1: // higher than threshold
        return 0.0;
    }
    return 0.0;
}
double regen_get_command(regen_data_t *reg)
{
    reg->previous_torque = _command_impl(reg, reg->speed_filtered[REG_FILTERED_BUFFER_SIZE - 1]);
    return reg->previous_torque;
}