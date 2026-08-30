/**
 * @file flight_log_format.h
 * @brief Binary flight log format (must match astra-loop/core/mcu_flight_log.py).
 */
#ifndef FLIGHT_LOG_FORMAT_H
#define FLIGHT_LOG_FORMAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLIGHT_LOG_BIN_MAGIC      ((uint32_t)0x4142494EU)
#define FLIGHT_LOG_BIN_VERSION    ((uint16_t)1U)
#define FLIGHT_LOG_RECORD_SIZE    ((uint16_t)32U)

typedef struct __attribute__((packed)) flight_log_record {
    uint32_t time_ms;
    uint8_t fc_state;
    uint8_t active;
    uint16_t servo_angle_cdeg;
    float position_z;
    float velocity_z;
    float velocity_lateral;
    float predicted_apogee;
    float time_to_apogee;
    float control_u;
} flight_log_record_t;

typedef struct __attribute__((packed)) flight_log_file_header {
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    uint32_t record_count;
} flight_log_file_header_t;

_Static_assert(sizeof(flight_log_record_t) == FLIGHT_LOG_RECORD_SIZE, "flight_log_record_t must be 32 bytes");
_Static_assert(sizeof(flight_log_file_header_t) == 12U, "flight_log_file_header_t must be 12 bytes");

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_LOG_FORMAT_H */
