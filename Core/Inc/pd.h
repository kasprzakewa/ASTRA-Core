/**
 * @file pd.h
 * @brief PD airbrake controller (mirrors PDController).
 */
#ifndef PD_H
#define PD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target_apogee; /* m */
    float kp;
    float kd;
    float error_deadband; /* m; u=0 if (predicted - target) < deadband */
} pd_params_t;

typedef struct {
    pd_params_t params;
    float prev_error;
    float prev_time;
    uint8_t has_prev;
} pd_t;

void pd_init(pd_t *pd);
void pd_reset(pd_t *pd);

/** Returns control u in [0, 1]. */
float pd_compute(pd_t *pd, float time_s, float predicted_apogee);

#ifdef __cplusplus
}
#endif

#endif /* PD_H */
