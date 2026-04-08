# Audio Packet Format

Audio packets transmit compressed audio data using **IMA ADPCM**.  
Each packet contains a **19-byte header** followed by **ADPCM audio data**.

---

# Header Structure

| Offset | Size | Field | Type | Endian | Description |
|------|------|------|------|------|-------------|
| 0 | 1 | `msg_type` | uint8 | — | Packet type identifier. Always `0x02` for audio packets. |
| 1 | 6 | `node_mac` | uint8[6] | — | MAC address of the sending node. |
| 7 | 2 | `seq_num` | uint16 | Little-Endian | Chunk index within the audio clip (0-based). |
| 9 | 2 | `total_chunks` | uint16 | Little-Endian | Total number of chunks required to reconstruct the clip. |
| 11 | 2 | `sample_rate` | uint16 | Little-Endian | Audio sample rate in Hz (e.g., `8000`, `16000`). |
| 13 | 4 | `total_samples` | uint32 | Little-Endian | Total number of PCM samples in the full recording. |
| 17 | 2 | `data_len` | uint16 | Little-Endian | Number of bytes of ADPCM data contained in this packet. |

**Total header size: 19 bytes**

---

# Payload Structure

| Offset | Size | Field | Type | Description |
|------|------|------|------|-------------|
| 19 | `data_len` | `audio_data` | uint8[] | IMA ADPCM compressed audio data |

---

# Packet Layout

```
|<------------- 19 byte header -------------->|<---- ADPCM data ---->|
+-----------+-----------+---------+----------+-----------+-----------+
| msg_type  | node_mac  | seq_num | chunks   | sample    | total     |
|           | (6 bytes) |         | total    | rate      | samples   |
+-----------+-----------+---------+----------+-----------+-----------+
| data_len  | ADPCM audio data...                                    |
+-----------+--------------------------------------------------------+
```


---

# Chunking and Reassembly

Audio recordings are transmitted in multiple packets.

To reconstruct a recording:

1. Group packets by **`node_mac`**
2. Collect packets until **`total_chunks`** are received
3. Sort packets by **`seq_num`**
4. Concatenate all `audio_data`
5. Decode **IMA ADPCM → PCM**
6. Save as a WAV file using:
   - `sample_rate`
   - `total_samples`

---

# Example Packet Metadata

| Field | Example |
|-----|-----|
| msg_type | `0x02` |
| node_mac | `D4:E9:F4:88:81:28` |
| seq_num | `3` |
| total_chunks | `12` |
| sample_rate | `8000` |
| total_samples | `35328` |
| data_len | `1451` |