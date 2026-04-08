#include "adpcm.h"

/* IMA ADPCM step size table (89 entries) */
static const int16_t step_table[89] = {
    7,     8,     9,    10,    11,    12,    13,    14,
   16,    17,    19,    21,    23,    25,    28,    31,
   34,    37,    41,    45,    50,    55,    60,    66,
   73,    80,    88,    97,   107,   118,   130,   143,
  157,   173,   190,   209,   230,   253,   279,   307,
  337,   371,   408,   449,   494,   544,   598,   658,
  724,   796,   876,   963,  1060,  1166,  1282,  1411,
 1552,  1707,  1878,  2066,  2272,  2499,  2749,  3024,
 3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,
 7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
32767
};

/* Index adjustment per nibble value */
static const int8_t index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

static inline int clamp(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static uint8_t encode_sample(int16_t sample, adpcm_state_t *state)
{
    int step = step_table[state->step_index];
    int diff = sample - state->predicted_value;

    uint8_t nibble = 0;
    if (diff < 0) {
        nibble = 8;
        diff = -diff;
    }

    if (diff >= step) { nibble |= 4; diff -= step; }
    if (diff >= step >> 1) { nibble |= 2; diff -= step >> 1; }
    if (diff >= step >> 2) { nibble |= 1; }

    /* Reconstruct the predicted value (must match decoder) */
    int pred_diff = (step >> 3);
    if (nibble & 4) pred_diff += step;
    if (nibble & 2) pred_diff += step >> 1;
    if (nibble & 1) pred_diff += step >> 2;

    if (nibble & 8) {
        state->predicted_value = (int16_t)clamp(state->predicted_value - pred_diff, -32768, 32767);
    } else {
        state->predicted_value = (int16_t)clamp(state->predicted_value + pred_diff, -32768, 32767);
    }

    state->step_index = (uint8_t)clamp(state->step_index + index_table[nibble], 0, 88);

    return nibble;
}

size_t adpcm_encode(const int16_t *pcm_in, size_t num_samples,
                    uint8_t *adpcm_out, adpcm_state_t *state)
{
    size_t out_bytes = 0;

    for (size_t i = 0; i < num_samples; i += 2) {
        uint8_t lo = encode_sample(pcm_in[i], state);
        uint8_t hi = (i + 1 < num_samples) ? encode_sample(pcm_in[i + 1], state) : 0;
        adpcm_out[out_bytes++] = (uint8_t)(lo | (hi << 4));
    }

    return out_bytes;
}
