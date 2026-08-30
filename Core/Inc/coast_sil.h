/**
 * @file coast_sil.h
 * @brief Closed-loop coast plant for MCU SIL (mirrors astra-loop/core/simulation_engine.py).
 */
#ifndef COAST_SIL_H
#define COAST_SIL_H

#include <stdint.h>

#include "control.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef COAST_SIL_MAX_STEPS_DEFAULT
#define COAST_SIL_MAX_STEPS_DEFAULT  (100000U)
#endif

typedef struct {
    float z;
    float vz;
    float v_h;
    float time_s;
    float u; /* [0,1], applied on next plant step */
    uint32_t steps;
    uint32_t max_steps;
    uint8_t done;
} coast_sil_t;

void coast_sil_init(
    coast_sil_t *sil,
    const flight_state_t *burnout_state,
    float initial_u,
    uint32_t max_steps);

/** Plant step with u -> predict(0) -> PD. Returns 0 when finished. */
uint8_t coast_sil_step(
    coast_sil_t *sil,
    control_t *ctl,
    flight_state_t *st_out,
    control_output_t *cmd_out);

#ifdef __cplusplus
}
#endif

#endif /* COAST_SIL_H */
