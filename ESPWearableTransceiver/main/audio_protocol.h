#pragma once

#include <stdint.h>

/*
 * Audio chunk protocol for ESP-NOW transmission.
 *
 * Each audio clip is split into sequential chunks that fit within the
 * 1470-byte ESP-NOW v2 payload limit.  The base station uses msg_type to
 * distinguish audio chunks from environmental JSON packets and from
 * acknowledgement / retransmit-request packets.
 */

#define AUDIO_MSG_TYPE        0x02
#define PROBE_MSG_TYPE        0x03
#define AUDIO_NACK_MSG_TYPE   0x04   /* Pi -> wearable: retransmit missing chunks */
#define AUDIO_ACK_MSG_TYPE    0x05   /* Pi -> wearable: session received in full   */
#define DUMP_REQUEST_MSG_TYPE 0x06   /* Pi -> wearable: dump queued offline sensor data */

#define ESPNOW_MAX_PAYLOAD    1470   /* ESP-NOW v2.0 max (ESP_NOW_MAX_DATA_LEN_V2) */
#define AUDIO_CHUNK_HDR_SIZE  23     /* sizeof(audio_chunk_header_t) */
#define AUDIO_CHUNK_DATA_MAX  (ESPNOW_MAX_PAYLOAD - AUDIO_CHUNK_HDR_SIZE)  /* 1447 */

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* Always AUDIO_MSG_TYPE (0x02)            */
    uint8_t  node_mac[6];     /* Sender MAC so base station knows who   */
    uint32_t session_id;      /* Unique per recording, for ack/nack     */
    uint16_t seq_num;         /* 0-based chunk index                    */
    uint16_t total_chunks;    /* Total chunks in this audio clip        */
    uint16_t sample_rate;     /* e.g. 8000                              */
    uint32_t total_samples;   /* Total PCM samples in the full clip     */
    uint16_t data_len;        /* Bytes of ADPCM data in this chunk      */
} audio_chunk_header_t;

_Static_assert(sizeof(audio_chunk_header_t) == AUDIO_CHUNK_HDR_SIZE,
               "audio_chunk_header_t size mismatch");

/*
 * NACK message sent from the Pi (via the basestation) to a specific wearable.
 * The wearable matches target_mac against its own MAC before acting.
 * missing_seqs is a variable-length array sized by missing_count.
 */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* Always AUDIO_NACK_MSG_TYPE (0x04) */
    uint8_t  target_mac[6];
    uint32_t session_id;
    uint16_t missing_count;
    /* uint16_t missing_seqs[missing_count] follows */
} audio_nack_msg_t;

#define AUDIO_NACK_HDR_SIZE   13
_Static_assert(sizeof(audio_nack_msg_t) == AUDIO_NACK_HDR_SIZE,
               "audio_nack_msg_t size mismatch");

/* Maximum missing-seq entries that fit in a single ESP-NOW v2 frame. */
#define AUDIO_NACK_MAX_SEQS   ((ESPNOW_MAX_PAYLOAD - AUDIO_NACK_HDR_SIZE) / sizeof(uint16_t))

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* Always AUDIO_ACK_MSG_TYPE (0x05) */
    uint8_t  target_mac[6];
    uint32_t session_id;
} audio_ack_msg_t;

#define AUDIO_ACK_SIZE        11
_Static_assert(sizeof(audio_ack_msg_t) == AUDIO_ACK_SIZE,
               "audio_ack_msg_t size mismatch");
