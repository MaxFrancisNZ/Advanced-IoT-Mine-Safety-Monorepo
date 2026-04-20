#!/usr/bin/env python3

import argparse
import serial
import struct
import time
import wave
import json
import os
from pathlib import Path
from dataclasses import dataclass, field
import httpx
from openai import OpenAI
from dotenv import load_dotenv
import firebase_admin
from firebase_admin import credentials, firestore, storage
from jsonschema import validate, ValidationError

load_dotenv()

UART_PORT = "/dev/serial0"
BAUD_RATE = 115200
TIMEOUT = 0.1

# --- Firebase config ---
FIREBASE_CREDENTIALS_PATH = os.getenv("FIREBASE_CREDENTIALS_PATH")
FIREBASE_COLLECTION = os.getenv("FIREBASE_COLLECTION", "iot-data")
FIREBASE_WARNINGS_COLLECTION = os.getenv("FIREBASE_WARNINGS_COLLECTION", "warnings")
FIREBASE_STORAGE_BUCKET = os.getenv("FIREBASE_STORAGE_BUCKET")
DEFAULT_SCHEMA_PATH = (
    Path(__file__).resolve().parent
    / "Advanced-IoT-Mine-Safety-Monorepo"
    / "node-data-schema.json"
)
SCHEMA_PATH = Path(os.getenv("SCHEMA_PATH", str(DEFAULT_SCHEMA_PATH)))

# 19-byte packed header:
# msg_type (0x02), node_mac[6], seq_num(uint16 LE), total_chunks(uint16 LE),
# sample_rate(uint16 LE), total_samples(uint32 LE), data_len(uint16 LE)
HEADER_FMT = "<B6sHHHIH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
AUDIO_MSG_TYPE = 0x02

DESKTOP_DIR = Path.home() / "Desktop"
OUTPUT_DIR = DESKTOP_DIR if DESKTOP_DIR.exists() else Path.home()
DEBUG_DIR = OUTPUT_DIR / "uart_audio_debug"
DEBUG_DIR.mkdir(parents=True, exist_ok=True)

# --- OpenAI config ---
# Set your API key via environment variable: export OPENAI_API_KEY="sk-..."
WHISPER_MODEL = "whisper-1"
CLASSIFY_MODEL = "gpt-4o-mini"
client: OpenAI = None  # initialised in main() after parsing CLI flags


def build_openai_client(no_proxy: bool) -> OpenAI:
    timeout = httpx.Timeout(60.0, connect=30.0)
    if no_proxy:
        for var in ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY",
                    "http_proxy", "https_proxy", "all_proxy"):
            os.environ.pop(var, None)
        http_client = httpx.Client(trust_env=False, timeout=timeout)
        return OpenAI(http_client=http_client, max_retries=2)
    return OpenAI(timeout=timeout, max_retries=2)

CLASSIFY_SYSTEM_PROMPT = """You are a safety classification system for short audio transcriptions (up to 5 seconds).
Classify the transcription into one of three warning levels:
- "safe": Normal speech, background noise, irrelevant/mundane content.
- "warning": Mildly concerning language, possible distress, ambiguous requests for help.
- "danger": Clear calls for help, emergency language, explicit distress signals.

You MUST respond with valid JSON only, in this exact format:
{"warning_level": "<safe|warning|danger>", "reasoning": "<brief explanation>"}"""


STEP_TABLE = [
     7,     8,     9,     10,    11,    12,    13,    14,
     16,    17,    19,    21,    23,    25,    28,    31,
     34,    37,    41,    45,    50,    55,    60,    66,
     73,    80,    88,    97,    107,   118,   130,   143,
     157,   173,   190,   209,   230,   253,   279,   307,
     337,   371,   408,   449,   494,   544,   598,   658,
     724,   796,   876,   963,   1060,  1166,  1282,  1411,
     1552,  1707,  1878,  2066,  2272,  2499,  2749,  3024,
     3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,
     7132,  7845,  8630,  9493,  10442, 11487, 12635, 13899,
     15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
     32767
]

INDEX_TABLE = [
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
]


class FirebaseClient:
    def __init__(self):
        self.enabled = False
        self.sensor_col = None
        self.warnings_col = None
        self.bucket = None
        self.schema = None

        if not FIREBASE_CREDENTIALS_PATH:
            print("[FIREBASE] disabled: FIREBASE_CREDENTIALS_PATH not set")
            return

        try:
            cred_path = Path(FIREBASE_CREDENTIALS_PATH)
            if not cred_path.is_absolute():
                cred_path = (Path(__file__).resolve().parent / cred_path).resolve()
            if not cred_path.is_file():
                raise FileNotFoundError(f"service account file not found: {cred_path}")
            print(f"[FIREBASE] using credentials: {cred_path}")
            cred = credentials.Certificate(str(cred_path))
            init_kwargs = {}
            if FIREBASE_STORAGE_BUCKET:
                init_kwargs["storageBucket"] = FIREBASE_STORAGE_BUCKET
            firebase_admin.initialize_app(cred, init_kwargs)

            db = firestore.client()
            self.sensor_col = db.collection(FIREBASE_COLLECTION)
            self.warnings_col = db.collection(FIREBASE_WARNINGS_COLLECTION)

            if FIREBASE_STORAGE_BUCKET:
                self.bucket = storage.bucket()
            else:
                print("[FIREBASE] storage disabled: FIREBASE_STORAGE_BUCKET not set")

            self._load_schema()

            self.enabled = True
            print(
                f"[FIREBASE] initialized (sensor='{FIREBASE_COLLECTION}', "
                f"warnings='{FIREBASE_WARNINGS_COLLECTION}', "
                f"bucket='{FIREBASE_STORAGE_BUCKET or 'none'}')"
            )
            self._test_connection()
        except Exception as e:
            print(f"[FIREBASE] disabled: init failed: {e}")
            self.enabled = False

    def _load_schema(self):
        try:
            with open(SCHEMA_PATH, "r") as f:
                self.schema = json.load(f)
            print(f"[FIREBASE] schema loaded from {SCHEMA_PATH}")
        except Exception as e:
            print(f"[FIREBASE] schema not loaded from {SCHEMA_PATH}: {e}")
            self.schema = None

    def _test_connection(self):
        """Round-trip a write to Firestore and check the bucket so failures surface at startup."""
        try:
            ping_ref = self.sensor_col.document("_connection_test")
            ping_ref.set({
                "ping": True,
                "tested_at": firestore.SERVER_TIMESTAMP,
                "tested_by": "pi-basestation",
            })
            ping_ref.delete()
            print(f"[FIREBASE] [OK] Firestore reachable (write+delete to {FIREBASE_COLLECTION}/_connection_test)")
        except Exception as e:
            print(f"[FIREBASE] [FAIL] Firestore test failed: {e}")
            print("[FIREBASE] (check service account project, IAM roles, and that Firestore is enabled)")

        if self.bucket is not None:
            try:
                exists = self.bucket.exists()
                if exists:
                    print(f"[FIREBASE] [OK] Storage bucket '{self.bucket.name}' reachable")
                else:
                    print(f"[FIREBASE] [FAIL] Storage bucket '{self.bucket.name}' does not exist or is not accessible")
            except Exception as e:
                print(f"[FIREBASE] [FAIL] Storage test failed: {e}")

    def publish_sensor_payload(self, payload: dict):
        """Validate and write a parsed sensor payload. Returns (status, detail).

        status is one of: "disabled", "invalid", "error", "ok".
        For "ok", detail is the new doc id. Otherwise detail is a human-readable message.
        """
        if not self.enabled or self.sensor_col is None:
            return ("disabled", "Firebase not enabled")

        if self.schema is not None:
            try:
                validate(instance=payload, schema=self.schema)
            except ValidationError as e:
                return ("invalid", e.message)

        try:
            doc_ref = self.sensor_col.add({
                **payload,
                "received_at": firestore.SERVER_TIMESTAMP,
                "received_by": "pi-basestation",
            })
            return ("ok", doc_ref[1].id)
        except Exception as e:
            return ("error", str(e))

    def upload_wav(self, wav_path: Path, mac_compact: str):
        if not self.enabled or self.bucket is None:
            return None
        try:
            blob_path = f"audio/{mac_compact}/{wav_path.name}"
            blob = self.bucket.blob(blob_path)
            blob.upload_from_filename(str(wav_path), content_type="audio/wav")
            blob.make_public()
            print(f"[FIREBASE] audio uploaded: gs://{self.bucket.name}/{blob_path}")
            return blob_path, blob.public_url
        except Exception as e:
            print(f"[FIREBASE] audio upload failed: {e}")
            return None

    def publish_warning(self, doc: dict):
        if not self.enabled or self.warnings_col is None:
            return None
        try:
            doc_ref = self.warnings_col.add({
                **doc,
                "received_at": firestore.SERVER_TIMESTAMP,
            })
            doc_id = doc_ref[1].id
            print(f"[FIREBASE] warning doc written: {FIREBASE_WARNINGS_COLLECTION}/{doc_id}")
            return doc_id
        except Exception as e:
            print(f"[FIREBASE] warning write failed: {e}")
            return None


@dataclass
class AudioSession:
    node_mac: bytes
    total_chunks: int
    sample_rate: int
    total_samples: int
    session_id: str
    session_dir: Path
    chunks: dict = field(default_factory=dict)
    started_at: float = field(default_factory=time.time)
    raw_stream_path: Path = None
    adpcm_concat_path: Path = None
    info_path: Path = None

    def add_chunk(self, seq_num: int, data: bytes):
        self.chunks[seq_num] = data

    def is_complete(self) -> bool:
        return self.total_chunks > 0 and len(self.chunks) == self.total_chunks

    def missing_chunks(self):
        return [i for i in range(self.total_chunks) if i not in self.chunks]


def mac_to_str(mac: bytes) -> str:
    return ":".join(f"{b:02X}" for b in mac)


def safe_text_from_bytes(data: bytes) -> str:
    return data.decode("utf-8", errors="replace").rstrip("\r\n")


def bytes_to_hex_preview(data: bytes, max_len: int = 64) -> str:
    preview = data[:max_len]
    hex_str = " ".join(f"{b:02X}" for b in preview)
    if len(data) > max_len:
        hex_str += " ..."
    return hex_str


def decode_ima_adpcm(adpcm):
    predictor = 0
    index = 0
    pcm = []

    for byte in adpcm:
        for nibble_shift in (0, 4):
            nibble = (byte >> nibble_shift) & 0x0F

            step = STEP_TABLE[index]

            diff = step >> 3
            if nibble & 1:
                diff += step >> 2
            if nibble & 2:
                diff += step >> 1
            if nibble & 4:
                diff += step
            if nibble & 8:
                predictor -= diff
            else:
                predictor += diff

            predictor = max(-32768, min(32767, predictor))

            index += INDEX_TABLE[nibble]
            index = max(0, min(88, index))

            pcm.append(predictor)

    return struct.pack("<" + "h" * len(pcm), *pcm)


def transcribe_audio(wav_path: Path) -> str:
    """Send a WAV file to the OpenAI Whisper API and return the transcript."""
    try:
        with open(wav_path, "rb") as audio_file:
            response = client.audio.transcriptions.create(
                model=WHISPER_MODEL,
                file=audio_file,
            )
        return response.text
    except Exception as e:
        print(f"[TRANSCRIBE] Error calling Whisper API: {e}")
        return None


def classify_transcription(transcript: str) -> dict:
    """Send a transcript to GPT-4o-mini to classify the warning level."""
    try:
        response = client.chat.completions.create(
            model=CLASSIFY_MODEL,
            response_format={"type": "json_object"},
            messages=[
                {"role": "system", "content": CLASSIFY_SYSTEM_PROMPT},
                {"role": "user", "content": transcript},
            ],
        )
        return json.loads(response.choices[0].message.content)
    except Exception as e:
        print(f"[CLASSIFY] Error calling classification API: {e}")
        return {"warning_level": "error", "reasoning": str(e)}


def write_wav_file(node_mac: bytes, sample_rate: int, total_samples: int, pcm_data: bytes) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    mac_str = mac_to_str(node_mac).replace(":", "")
    out_path = OUTPUT_DIR / f"audio_{mac_str}_{timestamp}.wav"

    expected_pcm_bytes = total_samples * 2
    if len(pcm_data) > expected_pcm_bytes:
        pcm_data = pcm_data[:expected_pcm_bytes]

    with wave.open(str(out_path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(pcm_data)

    return out_path


def create_session(node_mac: bytes, total_chunks: int, sample_rate: int, total_samples: int) -> AudioSession:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    mac_compact = mac_to_str(node_mac).replace(":", "")
    session_id = f"{mac_compact}_{timestamp}"
    session_dir = DEBUG_DIR / f"audio_debug_{session_id}"
    session_dir.mkdir(parents=True, exist_ok=True)

    session = AudioSession(
        node_mac=node_mac,
        total_chunks=total_chunks,
        sample_rate=sample_rate,
        total_samples=total_samples,
        session_id=session_id,
        session_dir=session_dir,
    )
    session.raw_stream_path = session_dir / "raw_packets_stream.bin"
    session.adpcm_concat_path = session_dir / "adpcm_payload_stream.bin"
    session.info_path = session_dir / "session_info.txt"

    with open(session.info_path, "w", encoding="utf-8") as f:
        f.write(f"session_id={session_id}\n")
        f.write(f"node_mac={mac_to_str(node_mac)}\n")
        f.write(f"total_chunks={total_chunks}\n")
        f.write(f"sample_rate={sample_rate}\n")
        f.write(f"total_samples={total_samples}\n")
        f.write(f"created_at={timestamp}\n")

    return session


class UARTAudioMonitor:
    def __init__(self, firebase: "FirebaseClient"):
        self.buffer = bytearray()
        self.sessions = {}
        self.firebase = firebase

    def handle_text_line(self, text: str):
        """Pretty-print a UART text line. If it's a JSON sensor payload, also submit it."""
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            # Not JSON — treat as ESP32 log output and pass through.
            print(text)
            return
        if not isinstance(payload, dict):
            print(text)
            return

        node_id = payload.get("node_id", "?")
        env = payload.get("environment", {}) or {}
        parts = [f"node={node_id}"]
        if isinstance(env.get("temperature"), (int, float)):
            parts.append(f"temp={env['temperature']:.1f}C")
        if isinstance(env.get("humidity"), (int, float)):
            parts.append(f"hum={env['humidity']:.1f}%")
        if isinstance(env.get("barometric_pressure"), (int, float)):
            parts.append(f"p={env['barometric_pressure']:.1f}hPa")
        accel = env.get("accelerometer") or {}
        if all(isinstance(accel.get(a), (int, float)) for a in ("x", "y", "z")):
            parts.append(f"accel=({accel['x']:.2f},{accel['y']:.2f},{accel['z']:.2f})")
        if isinstance(payload.get("battery_voltage"), (int, float)):
            parts.append(f"batt={payload['battery_voltage']:.2f}V")
        if "led_state" in payload:
            parts.append(f"led={payload['led_state']}")
        if "transmission_attempts" in payload:
            parts.append(f"tx={payload['transmission_attempts']}")
        print(f"[SENSOR] {'  '.join(parts)}")

        status, detail = self.firebase.publish_sensor_payload(payload)
        if status == "ok":
            print(f"[SENSOR]   -> Firestore OK: {FIREBASE_COLLECTION}/{detail}")
        elif status == "disabled":
            print(f"[SENSOR]   -> not submitted ({detail})")
        elif status == "invalid":
            print(f"[SENSOR]   -> schema validation failed: {detail}")
        elif status == "error":
            print(f"[SENSOR]   -> Firestore write FAILED: {detail}")

    def log_raw_packet(self, session: AudioSession, seq_num: int, packet: bytes, payload: bytes):
        packet_path = session.session_dir / f"chunk_{seq_num:05d}_packet.bin"
        payload_path = session.session_dir / f"chunk_{seq_num:05d}_adpcm.bin"
        meta_path = session.session_dir / f"chunk_{seq_num:05d}_meta.txt"

        with open(packet_path, "wb") as f:
            f.write(packet)

        with open(payload_path, "wb") as f:
            f.write(payload)

        with open(session.raw_stream_path, "ab") as f:
            f.write(packet)

        with open(session.adpcm_concat_path, "ab") as f:
            f.write(payload)

        with open(meta_path, "w", encoding="utf-8") as f:
            f.write(f"seq_num={seq_num}\n")
            f.write(f"packet_len={len(packet)}\n")
            f.write(f"payload_len={len(payload)}\n")
            f.write(f"packet_hex_preview={bytes_to_hex_preview(packet, 128)}\n")
            f.write(f"payload_hex_preview={bytes_to_hex_preview(payload, 128)}\n")

    def process_audio_packet(self, packet: bytes):
        header = packet[:HEADER_SIZE]
        payload = packet[HEADER_SIZE:]

        msg_type, node_mac, seq_num, total_chunks, sample_rate, total_samples, data_len = struct.unpack(
            HEADER_FMT, header
        )

        if msg_type != AUDIO_MSG_TYPE:
            return

        mac_key = mac_to_str(node_mac)

        session = self.sessions.get(mac_key)
        if session is None:
            session = create_session(node_mac, total_chunks, sample_rate, total_samples)
            self.sessions[mac_key] = session
            print(f"[AUDIO] Debug dump directory: {session.session_dir}")

        if seq_num == 0 and session.chunks:
            if not session.is_complete():
                print(f"[AUDIO] Restart detected from {mac_key}, discarding incomplete previous transfer")
            session = create_session(node_mac, total_chunks, sample_rate, total_samples)
            self.sessions[mac_key] = session
            print(f"[AUDIO] Debug dump directory: {session.session_dir}")

        if (
            session.total_chunks != total_chunks
            or session.sample_rate != sample_rate
            or session.total_samples != total_samples
        ):
            print(f"[AUDIO] Metadata changed mid-transfer for {mac_key}, resetting session")
            session = create_session(node_mac, total_chunks, sample_rate, total_samples)
            self.sessions[mac_key] = session
            print(f"[AUDIO] Debug dump directory: {session.session_dir}")

        if len(payload) != data_len:
            print(f"[AUDIO] Warning: payload length mismatch from {mac_key}: header={data_len}, actual={len(payload)}")

        self.log_raw_packet(session, seq_num, packet, payload)
        session.add_chunk(seq_num, payload)

        print(
            f"[AUDIO] {mac_key} chunk {seq_num + 1}/{total_chunks} "
            f"({len(payload)} bytes ADPCM, sample_rate={sample_rate}, total_samples={total_samples})"
        )
        print(f"[AUDIO] Header hex:  {bytes_to_hex_preview(header, HEADER_SIZE)}")
        print(f"[AUDIO] Payload hex: {bytes_to_hex_preview(payload, 64)}")

        if session.is_complete():
            self.finalize_session(mac_key)

    def finalize_session(self, mac_key: str):
        session = self.sessions.get(mac_key)
        if session is None:
            return

        ordered_adpcm = bytearray()
        missing = session.missing_chunks()
        if missing:
            print(f"[AUDIO] Cannot finalize {mac_key}, missing chunks: {missing}")
            return

        for i in range(session.total_chunks):
            ordered_adpcm.extend(session.chunks[i])

        assembled_adpcm_path = session.session_dir / "assembled_ordered_adpcm.bin"
        with open(assembled_adpcm_path, "wb") as f:
            f.write(ordered_adpcm)

        try:
            pcm = decode_ima_adpcm(bytes(ordered_adpcm))
            pcm_dump_path = session.session_dir / "decoded_pcm.raw"
            with open(pcm_dump_path, "wb") as f:
                f.write(pcm)

            out_path = write_wav_file(
                node_mac=session.node_mac,
                sample_rate=session.sample_rate,
                total_samples=session.total_samples,
                pcm_data=pcm,
            )
            with open(session.info_path, "a", encoding="utf-8") as f:
                f.write(f"assembled_adpcm={assembled_adpcm_path}\n")
                f.write(f"decoded_pcm={pcm_dump_path}\n")
                f.write(f"wav_file={out_path}\n")

            print(f"[AUDIO] Complete from {mac_key}")
            print(f"[AUDIO] Saved WAV: {out_path}")
            print(f"[AUDIO] Debug files: {session.session_dir}")

            # --- Transcribe via OpenAI Whisper API ---
            print(f"[TRANSCRIBE] Sending {out_path.name} to Whisper API...")
            transcript = transcribe_audio(out_path)
            if transcript is not None:
                print(f"[TRANSCRIBE] Result: {transcript}")

                # --- Classify warning level ---
                if transcript.strip():
                    print(f"[CLASSIFY] Sending transcript to {CLASSIFY_MODEL}...")
                    classification = classify_transcription(transcript)
                else:
                    classification = {"warning_level": "error", "reasoning": "Empty transcription"}

                print(f"[CLASSIFY] Warning level: {classification['warning_level']}")
                print(f"[CLASSIFY] Reasoning: {classification['reasoning']}")

                # Save transcript + classification to file
                transcript_path = out_path.with_suffix(".txt")
                result = {
                    "transcript": transcript,
                    "classification": classification,
                }
                with open(transcript_path, "w", encoding="utf-8") as f:
                    json.dump(result, f, indent=2)
                print(f"[TRANSCRIBE] Saved transcript + classification: {transcript_path}")

                with open(session.info_path, "a", encoding="utf-8") as f:
                    f.write(f"transcript_file={transcript_path}\n")
                    f.write(f"transcript={transcript}\n")
                    f.write(f"warning_level={classification['warning_level']}\n")

                # --- Publish to Firebase ---
                mac_str = mac_to_str(session.node_mac)
                mac_compact = mac_str.replace(":", "")
                storage_info = self.firebase.upload_wav(out_path, mac_compact)
                warning_doc = {
                    "mac_address": mac_str,
                    "node_id": mac_str,
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "session_id": session.session_id,
                    "transcript": transcript,
                    "classification": classification,
                    "sample_rate": session.sample_rate,
                    "total_samples": session.total_samples,
                    "audio_storage_path": storage_info[0] if storage_info else None,
                    "audio_download_url": storage_info[1] if storage_info else None,
                }
                self.firebase.publish_warning(warning_doc)
            else:
                print(f"[TRANSCRIBE] Transcription failed for {out_path.name}")

        except Exception as e:
            print(f"[AUDIO] Failed to decode/save audio from {mac_key}: {e}")

        del self.sessions[mac_key]

    def cleanup_stale_sessions(self, timeout_seconds: float = 30.0):
        now = time.time()
        stale = []
        for mac_key, session in self.sessions.items():
            if now - session.started_at > timeout_seconds:
                stale.append(mac_key)

        for mac_key in stale:
            session = self.sessions[mac_key]
            print(
                f"[AUDIO] Discarding stale incomplete session from {mac_key}, "
                f"missing chunks: {session.missing_chunks()}"
            )
            print(f"[AUDIO] Partial debug files kept in: {session.session_dir}")
            del self.sessions[mac_key]

    def process_buffer(self):
        while self.buffer:
            if self.buffer[0] == AUDIO_MSG_TYPE:
                if len(self.buffer) < HEADER_SIZE:
                    return

                try:
                    msg_type, node_mac, seq_num, total_chunks, sample_rate, total_samples, data_len = struct.unpack(
                        HEADER_FMT, self.buffer[:HEADER_SIZE]
                    )
                except struct.error:
                    return

                packet_len = HEADER_SIZE + data_len
                if len(self.buffer) < packet_len:
                    return

                packet = bytes(self.buffer[:packet_len])
                del self.buffer[:packet_len]
                self.process_audio_packet(packet)
                continue

            newline_index = self.buffer.find(b"\n")
            audio_index = self.buffer.find(bytes([AUDIO_MSG_TYPE]))

            if newline_index != -1 and (audio_index == -1 or newline_index < audio_index):
                line = bytes(self.buffer[:newline_index + 1])
                del self.buffer[:newline_index + 1]
                text = safe_text_from_bytes(line)
                if text:
                    self.handle_text_line(text)
                continue

            if audio_index > 0:
                text_fragment = bytes(self.buffer[:audio_index])
                del self.buffer[:audio_index]
                text = safe_text_from_bytes(text_fragment)
                if text:
                    self.handle_text_line(text)
                continue

            if len(self.buffer) > 4096:
                text = safe_text_from_bytes(bytes(self.buffer))
                if text:
                    print(text)
                self.buffer.clear()
            return


def main():
    global client

    parser = argparse.ArgumentParser(description="UART audio receiver with Whisper transcription")
    parser.add_argument(
        "--no-proxy",
        action="store_true",
        help="Ignore HTTP(S)_PROXY env vars and connect directly (use when off the office network).",
    )
    args = parser.parse_args()

    client = build_openai_client(no_proxy=args.no_proxy)
    if args.no_proxy:
        print("[NET] Proxy env vars ignored; connecting directly to OpenAI.")

    firebase_client = FirebaseClient()

    try:
        ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"Failed to open serial port {UART_PORT}: {e}")
        return

    print(f"Listening on {UART_PORT} at {BAUD_RATE} baud...")
    print(f"Audio files will be saved to: {OUTPUT_DIR}")
    print(f"Raw packet dumps will be saved to: {DEBUG_DIR}")

    monitor = UARTAudioMonitor(firebase_client)

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                monitor.buffer.extend(chunk)
                monitor.process_buffer()

            monitor.cleanup_stale_sessions()
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
