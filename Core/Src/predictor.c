/**
 * @file predictor.c
 * @brief 3D coast apogee predictor.
 */
#include "predictor.h"

#include <math.h>

void predictor_init(predictor_t *pred)
{
    model_params_default(&pred->params);
    pred->dt = 0.01f;
    pred->solver = SOLVER_EULER;
    pred->max_steps = 100000u;
}

predict_result_t predict_apogee_3d(
    const predictor_t *pred,
    float position_z,
    float velocity_z,
    float velocity_lateral,
    float control_signal)
{
    predict_result_t out = {0};

    if (isnan(position_z) || isnan(velocity_z)) {
        out.predicted_apogee = NAN;
        out.time_to_apogee = NAN;
        out.valid = 0;
        return out;
    }

    if (velocity_z <= 0.0f) {
        out.predicted_apogee = position_z;
        out.time_to_apogee = 0.0f;
        out.valid = 1;
        return out;
    }

    float servo = control_to_servo_angle(control_signal);
    float v_lat_sq = velocity_lateral * velocity_lateral;
    float tta = 0.0f;

    out.predicted_apogee = propagate_coast_to_apogee(
        position_z, velocity_z, v_lat_sq,
        &pred->params, pred->solver, pred->dt, pred->max_steps,
        servo, &tta);
    out.time_to_apogee = tta;
    out.valid = 1;
    return out;
}
