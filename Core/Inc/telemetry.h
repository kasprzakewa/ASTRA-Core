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
    TELEM_STATE_ARMED        = 1,
    TELEM_STATE_ACCELERATING = 2,
    TELEM_STATE_FREE_FLIGHT  = 3,
    TELEM_STATE_FREE_FALL    = 4,
    TELEM_STATE_LANDED       = 5,
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

/** Consecutive COAST+ 0<vz<=MAX_BRAKE_DEPLOY_SPEED_MS frames required before control_arm. */
#ifndef TELEM_ARM_COAST_FRAMES
#define TELEM_ARM_COAST_FRAMES  (3u)
#endif

void telemetry_to_flight_state(const stub_telemetry_frame_t *frame, flight_state_t *state);
uint8_t telemetry_is_coast(const stub_telemetry_frame_t *frame);

/** Map frame -> arm/disarm -> control_step. */
control_output_t telemetry_run_control(
    control_t *ctl,
    const stub_telemetry_frame_t *frame,
    flight_state_t *state_out);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
