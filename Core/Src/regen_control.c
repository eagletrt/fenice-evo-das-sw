#include "regen_control.h"

regen_data_t regen_data;

static int _thresholded_check(double value, double limit, double threshold)
{
    if (value < limit - threshold)
    {
        return -1;
    }
    if (value > limit + threshold)
    {
        return 1;
    }
    return 0;
}

void regen_control_init(regen_data_t *reg)
{
    reg->speed = 0.0;
    reg->speed_prev = 0.0;
    reg->speed_diff = 0.0;
    reg->speed_ref = 0.0;
    reg->torque_ref = 0.0;
    reg->previous_torque = 0.0;
    reg->acceleration_condition_should_regenerate = 0;
    reg->torque_cutoff = 0.0;
}
void regen_add_speed_sample_rads(regen_data_t *reg, double speed_rads)
{
    regen_add_speed_sample(reg, speed_rads * ENC_WHEEL_RADIUS);
}

float _cut_off_torque_impl(regen_data_t *reg)
{
    const float HV_MAX_REGEN_CURRENT = -24.0f;
    const float HV_MIN_CELL_VOLTAGE = 2.8f;
    const int HV_CELL_COUNT = 108;
    const float HV_MAX_REGEN_POWER = HV_MAX_REGEN_CURRENT * HV_MIN_CELL_VOLTAGE * HV_CELL_COUNT;
    
    float rpm = reg->speed / ENC_WHEEL_RADIUS * RADS_TO_RPM_COEFF;
    if(rpm > -0.5 && rpm < 0.5) {
        return reg->torque_cutoff;
    }
    return HV_MAX_REGEN_POWER / MOT_TORQUE_COEFF * RADS_TO_RPM_COEFF / rpm;
}
void regen_add_speed_sample(regen_data_t *reg, double speed_ms)
{
    reg->speed_prev = reg->speed;
    reg->speed = speed_ms * REG_FILTER_COEFFICIENT + reg->speed_prev * (1 - REG_FILTER_COEFFICIENT);
    reg->speed_diff = (reg->speed - reg->speed_prev) * REG_FILTER_COEFFICIENT + reg->speed_diff * (1 - REG_FILTER_COEFFICIENT);
    reg->torque_cutoff = _cut_off_torque_impl(reg);
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
    switch (_thresholded_check(reg->speed_diff, -REG_ACC_THRESHOLD, REG_ACC_THRESHOLD))
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
    double torque = _command_impl(reg, reg->speed);
    if(reg->torque_cutoff < 0) {
        reg->previous_torque = torque > reg->torque_cutoff ? torque : reg->torque_cutoff;
    }
    if(reg->previous_torque > 0) {
        reg->previous_torque = 0.0;
    }
    return reg->previous_torque;
}