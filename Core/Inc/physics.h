/**
 * @file physics.h
 * @brief Coast-flight physics
 */
#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FULL_SERVO_ANGLE_DEG  (9.16607764f)

static inline float clamp_f(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

typedef enum {
    SOLVER_EULER = 0,
    SOLVER_RK4   = 1
} coast_solver_t;

typedef struct {
    float reference_pressure;       /* Pa */
    float reference_air_density;    /* kg/m^3 */
    float reference_temperature;    /* K */
    float scale_height;             /* m */
    float baro_exponent;
    float density_exponent;
    float g;                        /* m/s^2 */
    float default_cross_section;    /* m^2 */
    float default_mass;             /* kg */
    float default_drag_coefficient; /* Cd0 rocket */
    float brake_drag_coefficient;   /* Cd brakes */
    float temperature_gradient;     /* K/m */
} model_params_t;

void model_params_default(model_params_t *p);

float control_to_servo_angle(float u);
float calculate_air_density(float altitude, const model_params_t *p);
float calculate_temperature(float altitude, const model_params_t *p);
float calculate_mach(float speed, float altitude, const model_params_t *p);
float calculate_drag_coefficient(float speed, float altitude, const model_params_t *p);
float calculate_brake_area(float servo_angle_deg);
float calculate_cross_section(const model_params_t *p, float servo_angle_deg);

void coast_derivatives(
    float z, float vz, float v_h,
    const model_params_t *p,
    float servo_angle_deg,
    float wind_speed,
    float wind_vertical,
    float *dz_dt, float *dvz_dt, float *dv_h_dt);

void step_coast(
    float *z, float *vz, float *v_h,
    const model_params_t *p,
    float dt,
    float servo_angle_deg,
    coast_solver_t solver,
    float wind_speed,
    float wind_vertical);

/** Integrate until vz <= 0; returns apogee [m]. */
float propagate_coast_to_apogee(
    float z, float vz, float v_lat_sq,
    const model_params_t *p,
    coast_solver_t solver,
    float dt,
    uint32_t max_steps,
    float servo_angle_deg,
    float *time_to_apogee);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_H */
