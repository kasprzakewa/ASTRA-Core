/**
 * @file control.h
 * @brief Guidance step: predict(u=0) -> PD -> servo angle.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include "pd.h"
#include "predictor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Max vertical speed [m/s] for brake deploy
 */
#ifndef MAX_BRAKE_DEPLOY_SPEED_MS
#define MAX_BRAKE_DEPLOY_SPEED_MS  200.0f
#endif

typedef struct {
    predictor_t predictor;
    pd_t pd;
    uint8_t armed;
} control_t;

typedef struct {
    float time_s;
    float position_z;
    float velocity_z;
    float velocity_lateral;
} flight_state_t;

typedef struct {
    float predicted_apogee;
    float time_to_apogee;
    float control_u; /* [0,1] */
    float servo_angle_deg;
    uint8_t active;
} control_output_t;

void control_init(control_t *ctl);
void control_arm(control_t *ctl);
void control_disarm(control_t *ctl);

/** Runs when armed && 0 < vz <= MAX_BRAKE_DEPLOY_SPEED_MS; forecast always uses control_signal=0. */
control_output_t control_step(control_t *ctl, const flight_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
