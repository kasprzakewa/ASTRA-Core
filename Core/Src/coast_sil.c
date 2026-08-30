/**
 * @file coast_sil.c
 * @brief Closed-loop coast plant for MCU SIL.
 */
#include "coast_sil.h"

#include "physics.h"

static float fabsf_local(float x)
{
    return (x < 0.0f) ? -x : x;
}

void coast_sil_init(
    coast_sil_t *sil,
    const flight_state_t *burnout_state,
    float initial_u,
    uint32_t max_steps)
{
    sil->z = burnout_state->position_z;
    sil->vz = burnout_state->velocity_z;
    sil->v_h = fabsf_local(burnout_state->velocity_lateral);
    sil->time_s = burnout_state->time_s;
    sil->u = clamp_f(initial_u, 0.0f, 1.0f);
    sil->steps = 0U;
    sil->max_steps = (max_steps == 0U) ? COAST_SIL_MAX_STEPS_DEFAULT : max_steps;
    sil->done = 0U;
}

uint8_t coast_sil_step(
    coast_sil_t *sil,
    control_t *ctl,
    flight_state_t *st_out,
    control_output_t *cmd_out)
{
    float dt;
    flight_state_t st;
    control_output_t cmd;

    if (sil->done) {
        return 0U;
    }
    if (sil->vz <= 0.0f || sil->steps >= sil->max_steps) {
        sil->done = 1U;
        return 0U;
    }

    dt = ctl->predictor.dt;
    if (dt <= 0.0f) {
        dt = 0.01f;
    }

    step_coast(
        &sil->z,
        &sil->vz,
        &sil->v_h,
        &ctl->predictor.params,
        dt,
        control_to_servo_angle(sil->u),
        ctl->predictor.solver,
        0.0f,
        0.0f);
    sil->time_s += dt;
    sil->steps++;

    st.time_s = sil->time_s;
    st.position_z = sil->z;
    st.velocity_z = sil->vz;
    st.velocity_lateral = fabsf_local(sil->v_h);

    control_arm(ctl);
    cmd = control_step(ctl, &st);
    sil->u = cmd.control_u;

    if (st_out != 0) {
        *st_out = st;
    }
    if (cmd_out != 0) {
        *cmd_out = cmd;
    }
    return 1U;
}
