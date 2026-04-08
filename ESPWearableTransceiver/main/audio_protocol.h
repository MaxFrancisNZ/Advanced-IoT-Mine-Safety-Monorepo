#pragma once

#include <stdint.h>

/*
 * Audio chunk protocol for ESP-NOW transmission.
 *
 * Each audio clip is split into sequential chunks that fit within the
 * 250-byte ESP-NOW payload limit.  The base station uses msg_type to
 * distinguish audio from environmental JSON packets.
 */

#define AUDIO_MSG_TYPE        0x02
#define ESPNOW_MAX_PAYLOAD    1470   /* ESP-NOW v2.0 max (ESP_NOW_MAX_DATA_LEN_V2) */
#define AUDIO_CHUNK_HDR_SIZE  19     /* sizeof(audio_chunk_header_t) */
#define AUDIO_CHUNK_DATA_MAX  (ESPNOW_MAX_PAYLOAD - AUDIO_CHUNK_HDR_SIZE)  /* 1451 */

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* Always AUDIO_MSG_TYPE (0x02)            */
    uint8_t  node_mac[6];     /* Sender MAC so base station knows who   */
    uint16_t seq_num;         /* 0-based chunk index                    */
    uint16_t total_chunks;    /* Total chunks in this audio clip        */
    uint16_t sample_rate;     /* e.g. 8000                              */
    uint32_t total_samples;   /* Total PCM samples in the full clip     */
    uint16_t data_len;        /* Bytes of ADPCM data in this chunk      */
} audio_chunk_header_t;

_Static_assert(sizeof(audio_chunk_header_t) == AUDIO_CHUNK_HDR_SIZE,
               "audio_chunk_header_t size mismatch");
