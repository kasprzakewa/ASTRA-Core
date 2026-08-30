/**
 * @file telemetry.c
 * @brief UART telemetry -> control.
 */
#include "telemetry.h"

#include <stddef.h>

_Static_assert(sizeof(stub_telemetry_frame_t) == 21u, "telemetry frame size mismatch");

static uint8_t s_coast_streak;

void telemetry_to_flight_state(
    const stub_telemetry_frame_t *frame,
    flight_state_t *state)
{
    state->time_s = (float)frame->timeMs * 0.001f;
    state->position_z = frame->posZ;
    state->velocity_z = frame->velZ;
    state->velocity_lateral = frame->gpsSpeed;
}

uint8_t telemetry_is_free_flight(const stub_telemetry_frame_t *frame)
{
    return frame->state == (uint8_t)TELEM_STATE_FREE_FLIGHT;
}

control_output_t telemetry_run_control(
    control_t *ctl,
    const stub_telemetry_frame_t *frame,
    flight_state_t *state_out)
{
    flight_state_t st;
    telemetry_to_flight_state(frame, &st);

    if (telemetry_is_free_flight(frame) &&
		st.velocity_z > 0.0f &&
		st.velocity_z <= MAX_BRAKE_DEPLOY_SPEED_MS)
    {
        if (s_coast_streak < 255u) {
            s_coast_streak++;
        }
        if (s_coast_streak >= (uint8_t)TELEM_ARM_COAST_FRAMES) {
            control_arm(ctl);
        } else {
            control_disarm(ctl);
        }
    } else {
        s_coast_streak = 0U;
        control_disarm(ctl);
    }

    control_output_t out = control_step(ctl, &st);
    if (state_out != NULL) {
        *state_out = st;
    }
    return out;
}
