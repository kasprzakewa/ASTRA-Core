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
    ctl->target_locked = 0;
    ctl->coast_streak = 0;
}

void control_arm(control_t *ctl)
{
    ctl->armed = 1;
}

void control_disarm(control_t *ctl)
{
    ctl->armed = 0;
    ctl->target_locked = 0;
    ctl->coast_streak = 0;
    pd_reset(&ctl->pd);
}

void control_update_coast_gate(
    control_t *ctl,
    float velocity_z,
    uint8_t phase_ok)
{
    if (phase_ok &&
        velocity_z > 0.0f &&
        velocity_z <= MAX_BRAKE_DEPLOY_SPEED_MS) {
        /* Once armed, stay armed while still eligible (no re-count / no wrap). */
        if (ctl->armed) {
            return;
        }
        if (ctl->coast_streak < 0xFFFFu) {
            ctl->coast_streak++;
        }
        if (ctl->coast_streak >= (uint16_t)TELEM_ARM_COAST_FRAMES) {
            control_arm(ctl);
        }
    } else {
        control_disarm(ctl);
    }
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

    if (!ctl->target_locked && DYNAMIC_TARGET_MARGIN_M > 0.0f) {
        ctl->pd.params.target_apogee =
            pred.predicted_apogee - DYNAMIC_TARGET_MARGIN_M;
        ctl->target_locked = 1;
    }

    out.control_u = pd_compute(&ctl->pd, state->time_s, pred.predicted_apogee);
    out.servo_angle_deg = control_to_servo_angle(out.control_u);
    out.active = 1;
    return out;
}
