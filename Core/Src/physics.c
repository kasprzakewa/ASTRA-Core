/**
 * @file physics.c
 * @brief Coast-flight physics.
 */
#include "physics.h"

#include <math.h>

void model_params_default(model_params_t *p)
{
    p->reference_pressure       = 101325.0f;
    p->reference_air_density    = 1.225f;
    p->reference_temperature    = 288.15f;
    p->scale_height             = 44330.0f;
    p->baro_exponent            = 0.1903f;
    p->density_exponent         = 4.256f;
    p->g                        = 9.80665f;
    p->default_cross_section    = 0.01f;
    p->default_mass             = 15.0f;
    p->default_drag_coefficient = 0.45f;
    p->brake_drag_coefficient   = 1.2f;
    p->temperature_gradient     = -0.0065f;
}

float control_to_servo_angle(float u)
{
    if (isnan(u)) {
        return 0.0f;
    }
    return clamp_f(u, 0.0f, 1.0f) * FULL_SERVO_ANGLE_DEG;
}

float calculate_air_density(float altitude, const model_params_t *p)
{
    if (altitude < 0.0f) {
        altitude = 0.0f;
    }
    float factor = 1.0f - altitude / p->scale_height;
    if (factor <= 0.0f) {
        return 0.0f;
    }
    return p->reference_air_density * powf(factor, p->density_exponent);
}

float calculate_temperature(float altitude, const model_params_t *p)
{
    if (altitude < 0.0f) {
        altitude = 0.0f;
    }
    return p->reference_temperature + p->temperature_gradient * altitude;
}

float calculate_mach(float speed, float altitude, const model_params_t *p)
{
    float temp = calculate_temperature(altitude, p);
    float local_sound_speed = 20.05f * sqrtf(temp);
    return speed / local_sound_speed;
}

float calculate_drag_coefficient(float speed, float altitude, const model_params_t *p)
{
    float mach = calculate_mach(speed, altitude, p);
    if (mach <= 0.0f) {
        return 0.0f;
    }
    float mach_sq = mach * mach;
    if (mach_sq > 0.99f) {
        mach_sq = 0.99f;
    }
    return p->default_drag_coefficient / sqrtf(1.0f - mach_sq);
}

float calculate_brake_area(float servo_angle_deg)
{
    /* Brake area polynomial [mm^2]*/
    float brake_mm2 =
        -86.89128f * servo_angle_deg * servo_angle_deg
        + 2284.145f * servo_angle_deg
        + 133.0275f;
    if (brake_mm2 < 0.0f) {
        brake_mm2 = 0.0f;
    }
    return brake_mm2 * 1e-6f;
}

float calculate_cross_section(const model_params_t *p, float servo_angle_deg)
{
    return p->default_cross_section + calculate_brake_area(servo_angle_deg);
}

void coast_derivatives(
    float z, float vz, float v_h,
    const model_params_t *p,
    float servo_angle_deg,
    float wind_speed,
    float wind_vertical,
    float *dz_dt, float *dvz_dt, float *dv_h_dt)
{
    float v_rel_h = v_h - wind_speed;
    float v_rel_z = vz - wind_vertical;
    float speed_air = sqrtf(v_rel_z * v_rel_z + v_rel_h * v_rel_h);
    float cd_body = calculate_drag_coefficient(speed_air, z, p);
    float rho = calculate_air_density(z, p);
    float a_brake = calculate_brake_area(servo_angle_deg);
    float cda = cd_body * p->default_cross_section
        + p->brake_drag_coefficient * a_brake;
    float k = -0.5f / p->default_mass * rho * cda * speed_air;

    *dz_dt   = vz;
    *dvz_dt  = -p->g + k * v_rel_z;
    *dv_h_dt = k * v_rel_h;
}

void step_coast(
    float *z, float *vz, float *v_h,
    const model_params_t *p,
    float dt,
    float servo_angle_deg,
    coast_solver_t solver,
    float wind_speed,
    float wind_vertical)
{
    if (solver == SOLVER_RK4) {
        float k1z, k1vz, k1vh;
        float k2z, k2vz, k2vh;
        float k3z, k3vz, k3vh;
        float k4z, k4vz, k4vh;

        coast_derivatives(*z, *vz, *v_h, p, servo_angle_deg,
                          wind_speed, wind_vertical, &k1z, &k1vz, &k1vh);
        coast_derivatives(
            *z + 0.5f * dt * k1z, *vz + 0.5f * dt * k1vz, *v_h + 0.5f * dt * k1vh,
            p, servo_angle_deg, wind_speed, wind_vertical, &k2z, &k2vz, &k2vh);
        coast_derivatives(
            *z + 0.5f * dt * k2z, *vz + 0.5f * dt * k2vz, *v_h + 0.5f * dt * k2vh,
            p, servo_angle_deg, wind_speed, wind_vertical, &k3z, &k3vz, &k3vh);
        coast_derivatives(
            *z + dt * k3z, *vz + dt * k3vz, *v_h + dt * k3vh,
            p, servo_angle_deg, wind_speed, wind_vertical, &k4z, &k4vz, &k4vh);

        *z   += (dt / 6.0f) * (k1z  + 2.0f * k2z  + 2.0f * k3z  + k4z);
        *vz  += (dt / 6.0f) * (k1vz + 2.0f * k2vz + 2.0f * k3vz + k4vz);
        *v_h += (dt / 6.0f) * (k1vh + 2.0f * k2vh + 2.0f * k3vh + k4vh);
        return;
    }

    float dz_dt, dvz_dt, dv_h_dt;
    coast_derivatives(*z, *vz, *v_h, p, servo_angle_deg,
                      wind_speed, wind_vertical, &dz_dt, &dvz_dt, &dv_h_dt);
    *z   += dz_dt * dt;
    *vz  += dvz_dt * dt;
    *v_h += dv_h_dt * dt;
}

float propagate_coast_to_apogee(
    float z, float vz, float v_lat_sq,
    const model_params_t *p,
    coast_solver_t solver,
    float dt,
    uint32_t max_steps,
    float servo_angle_deg,
    float *time_to_apogee)
{
    if (vz <= 0.0f) {
        if (time_to_apogee) {
            *time_to_apogee = 0.0f;
        }
        return z;
    }

    float v_h = sqrtf(v_lat_sq > 0.0f ? v_lat_sq : 0.0f);
    uint32_t steps = 0;

    while (vz > 0.0f && steps < max_steps) {
        step_coast(&z, &vz, &v_h, p, dt, servo_angle_deg, solver, 0.0f, 0.0f);
        steps++;
    }

    if (time_to_apogee) {
        *time_to_apogee = (float)steps * dt;
    }
    return z;
}
