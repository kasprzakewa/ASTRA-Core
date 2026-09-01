/**
 * @file control.c
 * @brief Guidance step: predict(u=0) -> PD -> servo.
 */
#include "control.h"

void control_init(control_t *ctl)
{
    predictor_init(&ctl->predictor);
    pd_init(&ctl->pd);
    ctl->armed = 0;
}

void control_arm(control_t *ctl)
{
    ctl->armed = 1;
}

void control_disarm(control_t *ctl)
{
    ctl->armed = 0;
    pd_reset(&ctl->pd);
}

control_output_t control_step(
    control_t *ctl,
    const flight_state_t *state)
{
    control_output_t out = {0};

    if (!ctl->armed ||
        state->velocity_z <= 0.0f ||
        state->velocity_z > MAX_BRAKE_DEPLOY_SPEED_MS) {
        out.control_u = 0.0f;
        out.servo_angle_deg = servo_angle_safe_deg();
        out.active = 0;
        if (state->velocity_z > 0.0f) {
            predict_result_t pred = predict_apogee_3d(
                &ctl->predictor,
                state->position_z,
                state->velocity_z,
                state->velocity_lateral,
                0.0f);
            out.predicted_apogee = pred.predicted_apogee;
            out.time_to_apogee = pred.time_to_apogee;
        } else {
            out.predicted_apogee = state->position_z;
            out.time_to_apogee = 0.0f;
        }
        return out;
    }

    /* Forecast with brakes closed (control_signal = 0). */
    predict_result_t pred = predict_apogee_3d(
        &ctl->predictor,
        state->position_z,
        state->velocity_z,
        state->velocity_lateral,
        0.0f);

    out.predicted_apogee = pred.predicted_apogee;
    out.time_to_apogee = pred.time_to_apogee;
    out.control_u = pd_compute(&ctl->pd, state->time_s, pred.predicted_apogee);
    out.servo_angle_deg = control_to_servo_angle(out.control_u);
    out.active = 1;
    return out;
}
