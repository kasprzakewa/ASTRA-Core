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
 * Max vertical speed [m/s] for brake deploy (~400 N envelope:
 * v = sqrt(2*F/(rho*Cx*A)), rho=1.2, Cx=1.2, A=0.0138 -> ~200.6 m/s).
 */
#ifndef MAX_BRAKE_DEPLOY_SPEED_MS
#define MAX_BRAKE_DEPLOY_SPEED_MS  200.0f
#endif

/**
 * Consecutive FREE_FLIGHT (state 2) samples with 0 < vz <= MAX before arm,
 * dynamic target lock (pred − margin), and PD. Same for UART and playback.
 * ~900 ≈ burnout→~7 s on Meteor nominal CSV (~2–3 ms/sample).
 */
#ifndef TELEM_ARM_COAST_FRAMES
#define TELEM_ARM_COAST_FRAMES  900u
#endif

/** On arm: target = predicted_apogee - margin. 0 = use fixed PD target. */
#ifndef DYNAMIC_TARGET_MARGIN_M
#define DYNAMIC_TARGET_MARGIN_M  200.0f
#endif

typedef struct {
    predictor_t predictor;
    pd_t pd;
    uint8_t armed;
    uint8_t target_locked;
    uint16_t coast_streak;
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

/**
 * Update coast_streak / armed from one sample.
 * phase_ok: FREE_FLIGHT on UART, 1 during playback coast SIL.
 */
void control_update_coast_gate(
    control_t *ctl,
    float velocity_z,
    uint8_t phase_ok);

/** Runs when armed && 0 < vz <= MAX_BRAKE_DEPLOY_SPEED_MS; forecast uses u=0. */
control_output_t control_step(control_t *ctl, const flight_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
