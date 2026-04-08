#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int16_t predicted_value;
    uint8_t step_index;
} adpcm_state_t;

/**
 * Encode PCM samples to IMA ADPCM.
 *
 * @param pcm_in     Input buffer of 16-bit signed PCM samples.
 * @param num_samples Number of samples to encode.
 * @param adpcm_out  Output buffer. Must be at least (num_samples + 1) / 2 bytes.
 * @param state      Encoder state (caller-owned, zero-initialise before first call).
 * @return Number of bytes written to adpcm_out.
 */
size_t adpcm_encode(const int16_t *pcm_in, size_t num_samples,
                    uint8_t *adpcm_out, adpcm_state_t *state);
