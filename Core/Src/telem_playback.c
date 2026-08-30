/**
 * @file telem_playback.c
 * @brief Cursor over telem_playback_frames[].
 */
#include "telem_playback.h"

#include <string.h>

static uint32_t s_idx;

void telem_playback_reset(void)
{
    s_idx = 0U;
}

uint8_t telem_playback_next(stub_telemetry_frame_t *out)
{
    if (out == 0 || s_idx >= TELEM_PLAYBACK_FRAME_COUNT) {
        return 0U;
    }
    memcpy(out, &telem_playback_frames[s_idx], sizeof(*out));
    s_idx++;
    return 1U;
}

uint32_t telem_playback_burnout_index(void)
{
    uint32_t i;
    uint32_t peak = 0U;
    float peak_vz = 0.0f;
    uint8_t found_start = 0U;

    for (i = 0U; i < TELEM_PLAYBACK_FRAME_COUNT; i++) {
        const float vz = telem_playback_frames[i].velZ;
        if (!found_start) {
            if (vz > 0.0f) {
                found_start = 1U;
                peak = i;
                peak_vz = vz;
            }
            continue;
        }
        if (vz > peak_vz) {
            peak_vz = vz;
            peak = i;
        }
    }

    if (!found_start || peak_vz <= 0.0f) {
        return TELEM_PLAYBACK_FRAME_COUNT;
    }
    return peak;
}

uint32_t telem_playback_index(void)
{
    return s_idx;
}
