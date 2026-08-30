/**
 * @file flight_log.h
 * @brief Binary flight log in internal flash (live write or RAM staging + flush).
 */
#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <stdint.h>

#include "control.h"
#include "flight_log_format.h"
#include "telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLIGHT_LOG_FLASH_HEADER_SIZE  (32U)

#ifndef FLIGHT_LOG_RAM_CAPACITY
#define FLIGHT_LOG_RAM_CAPACITY  (8190U)
#endif

/**
 * Flash error LEDs; activity LEDs are in main:
 *   YELLOW on (main)    — init done, main loop running
 *   GREEN toggle (main) — UART frame received
 *   RED on (here/main)  — erase/program/verify or append failed
 */
#ifndef FLIGHT_LOG_DEBUG_LEDS
#define FLIGHT_LOG_DEBUG_LEDS  (1)
#endif

/** Resume existing log or write header (erases log region first if header slot not 0xFF). */
int flight_log_boot(void);

/** RAM staging for offline playback SIL (flush at end). */
int flight_log_begin_staging(void);

int flight_log_append(
    const stub_telemetry_frame_t *frame,
    const flight_state_t *st,
    const control_output_t *cmd);

int flight_log_flush(void);

uint8_t flight_log_is_full(void);
uint32_t flight_log_record_count(void);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_LOG_H */
