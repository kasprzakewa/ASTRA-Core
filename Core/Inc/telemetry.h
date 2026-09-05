/**
 * @file telemetry.h
 * @brief UART telemetry frame -> control inputs.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include "control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TELEM_STATE_STANDING     = 0,
    TELEM_STATE_ACCELERATING = 1,
    TELEM_STATE_FREE_FLIGHT  = 2,
    TELEM_STATE_FREE_FALL    = 3,
    TELEM_STATE_LANDED       = 4,
} telemetry_state_t;

typedef struct __attribute__((packed)) stub_telemetry_frame {
    uint8_t state;
    uint32_t timeMs;
    float posZ;      /* m */
    float velZ;      /* m/s */
    float gpsSpeed;  /* m/s horizontal */
    float gpsCourse; /* deg, unused */
} stub_telemetry_frame_t;

#define TELEMETRY_FRAME_SIZE  (sizeof(stub_telemetry_frame_t))

/** Arm / dynamic-target lock after TELEM_ARM_COAST_FRAMES (defined in control.h). */
void telemetry_to_flight_state(const stub_telemetry_frame_t *frame, flight_state_t *state);
uint8_t telemetry_is_free_flight(const stub_telemetry_frame_t *frame);

/** Map frame -> arm/disarm -> control_step. */
control_output_t telemetry_run_control(
    control_t *ctl,
    const stub_telemetry_frame_t *frame,
    flight_state_t *state_out);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
