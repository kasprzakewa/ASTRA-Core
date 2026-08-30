/**
 * @file telem_playback.h
 * @brief Embedded CSV frames for offline MCU testing.
 */
#ifndef TELEM_PLAYBACK_H
#define TELEM_PLAYBACK_H

#include <stdint.h>

#include "telem_playback_data.h"
#include "telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

void telem_playback_reset(void);

/** Copy next frame; returns 0 when exhausted. */
uint8_t telem_playback_next(stub_telemetry_frame_t *out);

/** Peak velZ index (desktop find_burnout_index); frame count if none. */
uint32_t telem_playback_burnout_index(void);

uint32_t telem_playback_index(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEM_PLAYBACK_H */
