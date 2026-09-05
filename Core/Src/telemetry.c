/**
 * @file telemetry.c
 * @brief UART telemetry -> control.
 */
#include "telemetry.h"

#include <stddef.h>

_Static_assert(sizeof(stub_telemetry_frame_t) == 21u, "telemetry frame size mismatch");

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

    control_update_coast_gate(
        ctl,
        st.velocity_z,
        telemetry_is_free_flight(frame));

    control_output_t out = control_step(ctl, &st);
    if (state_out != NULL) {
        *state_out = st;
    }
    return out;
}
