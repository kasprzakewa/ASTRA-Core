/**
 * @file pd.c
 * @brief PD airbrake controller.
 */
#include "pd.h"
#include "physics.h"

void pd_init(pd_t *pd)
{
    pd->params.target_apogee = 2500.0f;
    pd->params.kp = 0.02f;
    pd->params.kd = 0.0001f;
    pd->params.error_deadband = 30.0f;
    pd_reset(pd);
}

void pd_reset(pd_t *pd)
{
    pd->prev_error = 0.0f;
    pd->prev_time = 0.0f;
    pd->has_prev = 0;
}

float pd_compute(pd_t *pd, float time_s, float predicted_apogee)
{
    float error = predicted_apogee - pd->params.target_apogee;
    float de = 0.0f;

    if (error < pd->params.error_deadband) {
		return 0.0f;
	}

    if (pd->has_prev) {
        float dt = time_s - pd->prev_time;
        if (dt > 0.0f) {
            de = (error - pd->prev_error) / dt;
        }
    }

    float u = clamp_f(pd->params.kp * error + pd->params.kd * de, 0.0f, 1.0f);
    pd->prev_error = error;
    pd->prev_time = time_s;
    pd->has_prev = 1;
    return u;
}
