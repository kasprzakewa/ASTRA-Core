/**
 * @file predictor.h
 * @brief 3D coast apogee predictor (mirrors ApogeePredict3D).
 */
#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "physics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    model_params_t params;
    float dt; /* s */
    coast_solver_t solver;
    uint32_t max_steps;
} predictor_t;

typedef struct {
    float predicted_apogee; /* m */
    float time_to_apogee;   /* s */
    uint8_t valid;
} predict_result_t;

void predictor_init(predictor_t *pred);

/** Forecast apogee; for control pass control_signal = 0. */
predict_result_t predict_apogee_3d(
    const predictor_t *pred,
    float position_z,
    float velocity_z,
    float velocity_lateral,
    float control_signal);

#ifdef __cplusplus
}
#endif

#endif /* PREDICTOR_H */
