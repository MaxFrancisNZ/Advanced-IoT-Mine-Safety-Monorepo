#!/usr/bin/env python3

import argparse
import re
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

# --- Audio processing config ---
# Peak-normalise decoded audio before saving/transcribing.
# Set NORMALIZE_AUDIO=0 to disable.
NORMALIZE_AUDIO = os.getenv("NORMALIZE_AUDIO", "1").lower() not in ("0", "false", "no", "off")
NORMALIZE_TARGET_PEAK = int(os.getenv("NORMALIZE_TARGET_PEAK", "30000"))
NORMALIZE_MAX_GAIN = float(os.getenv("NORMALIZE_MAX_GAIN", "8.0"))

# --- Firebase config ---
FIREBASE_CREDENTIALS_PATH = os.getenv("FIREBASE_CREDENTIALS_PATH")
FIREBASE_COLLECTION = os.getenv("FIREBASE_COLLECTION", "iot-data")
FIREBASE_WARNINGS_COLLECTION = os.getenv("FIREBASE_WARNINGS_COLLECTION", "warnings")
FIREBASE_WEATHER_COLLECTION = os.getenv("FIREBASE_WEATHER_COLLECTION", "weather_reference")
FIREBASE_STORAGE_BUCKET = os.getenv("FIREBASE_STORAGE_BUCKET")

METSERVICE_OBSERVATIONS_URL = "https://www.metservice.com/publicData/webdata/maps-radar/3-hourly-observations"
METSERVICE_FETCH_INTERVAL_S = 3600  # 1 hour
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

# --- Offline Firebase fallback config ---
# When Firestore/Storage is unavailable, records are written here as JSON files.
FIREBASE_FALLBACK_DIR = Path(
    os.getenv("FIREBASE_FALLBACK_DIR", str(OUTPUT_DIR / "firebase_offline_queue"))
)

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


def utc_now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def fetch_auckland_airport_pressure():
    """
    Scrape MetService 3-hourly observations for Auckland Airport pressure.
    Returns {pressure_hpa, source_url, fetched_at} on success, or None.
    """
    try:
        resp = httpx.get(METSERVICE_OBSERVATIONS_URL, timeout=20.0)
        resp.raise_for_status()
        text = resp.text
    except Exception as e:
        print(f"[WEATHER] fetch failed: {e}")
        return None

    match = re.search(
        r"Auckland\s*\(Airport\).*?(\d{3,4})\s*hPa",
        text,
        flags=re.DOTALL | re.IGNORECASE,
    )
    if not match:
        print("[WEATHER] Auckland (Airport) pressure not found in MetService response")
        return None

    try:
        pressure = int(match.group(1))
    except ValueError:
        print(f"[WEATHER] could not parse pressure value: {match.group(1)!r}")
        return None

    if not (900 <= pressure <= 1100):
        print(f"[WEATHER] pressure value {pressure}hPa outside plausible range, ignoring")
        return None

    return {
        "station": "Auckland (Airport)",
        "pressure_hpa": pressure,
        "source_url": METSERVICE_OBSERVATIONS_URL,
        "fetched_at": utc_now_iso(),
    }


def update_weather_reference(firebase_client) -> bool:
    data = fetch_auckland_airport_pressure()
    if not data:
        return False
    return firebase_client.publish_weather_reference("auckland_airport", data)


class FirebaseFallbackStore:
    """Local append-only queue for Firebase operations that could not be sent."""

    def __init__(self, queue_dir: Path):
        self.queue_dir = queue_dir
        self.queue_dir.mkdir(parents=True, exist_ok=True)
        self.queue_path = self.queue_dir / "firebase_upload_queue.jsonl"
        self.export_path = self.queue_dir / "firestore_offline_export.json"
        self.readme_path = self.queue_dir / "README.txt"
        self._write_readme()
        self._refresh_export()

    def _write_readme(self):
        if self.readme_path.exists():
            return
        self.readme_path.write_text(
            "Firebase offline fallback files\n"
            "===============================\n\n"
            "This folder is created when the Raspberry Pi cannot reach Firebase, "
            "or when an individual Firestore/Storage operation fails.\n\n"
            "Files:\n"
            "- firebase_upload_queue.jsonl: append-only queue; one JSON operation per line.\n"
            "- firestore_offline_export.json: easier-to-read export grouped by Firestore collection.\n\n"
            "Operation types:\n"
            "- firestore_add: add the supplied document to the named Firestore collection.\n"
            "- storage_upload: upload the local file to the supplied Firebase Storage path.\n\n"
            "The local queue is not deleted automatically. After uploading/importing the data, "
            "archive or delete the queue files yourself to avoid duplicate uploads.\n",
            encoding="utf-8",
        )

    def _json_safe(self, value):
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, dict):
            return {str(k): self._json_safe(v) for k, v in value.items()}
        if isinstance(value, list):
            return [self._json_safe(v) for v in value]
        if isinstance(value, tuple):
            return [self._json_safe(v) for v in value]
        return value

    def _append(self, record: dict) -> str:
        record = self._json_safe(record)
        with open(self.queue_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._refresh_export()
        return str(self.queue_path)

    def _read_queue(self) -> list:
        if not self.queue_path.exists():
            return []
        records = []
        with open(self.queue_path, "r", encoding="utf-8") as f:
            for line_no, line in enumerate(f, start=1):
                line = line.strip()
                if not line:
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as e:
                    print(f"[FIREBASE-FALLBACK] skipped invalid queue line {line_no}: {e}")
        return records

    def _refresh_export(self):
        records = self._read_queue()
        collections = {}
        storage_uploads = []
        for record in records:
            if record.get("operation") == "firestore_add":
                collections.setdefault(record.get("collection", "unknown"), []).append(record.get("document", {}))
            elif record.get("operation") == "storage_upload":
                storage_uploads.append({
                    "bucket": record.get("bucket"),
                    "storage_path": record.get("storage_path"),
                    "local_path": record.get("local_path"),
                    "content_type": record.get("content_type"),
                    "queued_at": record.get("queued_at"),
                })

        export = {
            "generated_at": utc_now_iso(),
            "queue_file": str(self.queue_path),
            "collections": collections,
            "storage_uploads": storage_uploads,
        }
        with open(self.export_path, "w", encoding="utf-8") as f:
            json.dump(export, f, indent=2, ensure_ascii=False)

    def queue_firestore_add(self, collection: str, document: dict, reason: str) -> str:
        queue_file = self._append({
            "operation": "firestore_add",
            "collection": collection,
            "document": document,
            "queued_at": utc_now_iso(),
            "reason": reason,
        })
        print(f"[FIREBASE-FALLBACK] queued Firestore document for '{collection}': {queue_file}")
        return queue_file

    def queue_storage_upload(
        self,
        local_path: Path,
        storage_path: str,
        content_type: str = "application/octet-stream",
        bucket: str = None,
        reason: str = "Firebase Storage unavailable",
    ) -> str:
        queue_file = self._append({
            "operation": "storage_upload",
            "bucket": bucket,
            "storage_path": storage_path,
            "local_path": str(local_path),
            "content_type": content_type,
            "queued_at": utc_now_iso(),
            "reason": reason,
        })
        print(f"[FIREBASE-FALLBACK] queued Storage upload '{storage_path}': {queue_file}")
        return queue_file


class FirebaseClient:
    def __init__(self):
        self.enabled = False
        self.firestore_online = False
        self.storage_online = False
        self.sensor_col = None
        self.warnings_col = None
        self.weather_col = None
        self.bucket = None
        self.schema = None
        self.fallback = FirebaseFallbackStore(FIREBASE_FALLBACK_DIR)
        print(f"[FIREBASE-FALLBACK] offline queue: {self.fallback.queue_path}")
        print(f"[FIREBASE-FALLBACK] grouped export: {self.fallback.export_path}")

        if not FIREBASE_CREDENTIALS_PATH:
            print("[FIREBASE] disabled: FIREBASE_CREDENTIALS_PATH not set")
            self._load_schema()
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
            self.weather_col = db.collection(FIREBASE_WEATHER_COLLECTION)

            if FIREBASE_STORAGE_BUCKET:
                self.bucket = storage.bucket()
            else:
                print("[FIREBASE] storage disabled: FIREBASE_STORAGE_BUCKET not set")

            self._load_schema()

            print(
                f"[FIREBASE] initialized (sensor='{FIREBASE_COLLECTION}', "
                f"warnings='{FIREBASE_WARNINGS_COLLECTION}', "
                f"bucket='{FIREBASE_STORAGE_BUCKET or 'none'}')"
            )
            self.firestore_online, self.storage_online = self._test_connection()
            self.enabled = self.firestore_online
            if not self.firestore_online:
                print("[FIREBASE] Firestore unavailable; new Firestore records will be written to the offline queue")
            if self.bucket is not None and not self.storage_online:
                print("[FIREBASE] Storage unavailable; new audio uploads will be written to the offline queue")
        except Exception as e:
            print(f"[FIREBASE] disabled: init failed: {e}")
            print("[FIREBASE] new Firebase records will be written to the offline queue")
            self.enabled = False
            self.firestore_online = False
            self.storage_online = False

    def _load_schema(self):
        try:
            with open(SCHEMA_PATH, "r") as f:
                self.schema = json.load(f)
            print(f"[FIREBASE] schema loaded from {SCHEMA_PATH}")
        except Exception as e:
            print(f"[FIREBASE] schema not loaded from {SCHEMA_PATH}: {e}")
            self.schema = None

    def _test_connection(self):
        """Round-trip Firestore and Storage tests so failures surface at startup."""
        firestore_ok = False
        storage_ok = False

        try:
            ping_ref = self.sensor_col.document("_connection_test")
            ping_ref.set({
                "ping": True,
                "tested_at": firestore.SERVER_TIMESTAMP,
                "tested_by": "pi-basestation",
            })
            ping_ref.delete()
            firestore_ok = True
            print(f"[FIREBASE] [OK] Firestore reachable (write+delete to {FIREBASE_COLLECTION}/_connection_test)")
        except Exception as e:
            print(f"[FIREBASE] [FAIL] Firestore test failed: {e}")
            print("[FIREBASE] (check service account project, IAM roles, and that Firestore is enabled)")

        if self.bucket is not None:
            try:
                exists = self.bucket.exists()
                if exists:
                    storage_ok = True
                    print(f"[FIREBASE] [OK] Storage bucket '{self.bucket.name}' reachable")
                else:
                    print(f"[FIREBASE] [FAIL] Storage bucket '{self.bucket.name}' does not exist or is not accessible")
            except Exception as e:
                print(f"[FIREBASE] [FAIL] Storage test failed: {e}")

        return firestore_ok, storage_ok

    def _sensor_document(self, payload: dict, online: bool) -> dict:
        if online:
            return {
                **payload,
                "received_at": firestore.SERVER_TIMESTAMP,
                "received_by": "pi-basestation",
            }
        return {
            **payload,
            "received_at": utc_now_iso(),
            "received_by": "pi-basestation",
            "offline_queued": True,
        }

    def _warning_document(self, doc: dict, online: bool) -> dict:
        if online:
            return {
                **doc,
                "received_at": firestore.SERVER_TIMESTAMP,
            }
        return {
            **doc,
            "received_at": utc_now_iso(),
            "offline_queued": True,
        }

    def publish_sensor_payload(self, payload: dict):
        """Validate and write a parsed sensor payload. Returns (status, detail).

        status is one of: "queued", "invalid", "error", "ok".
        For "ok", detail is the new doc id. For "queued", detail is the local queue file.
        """
        if self.schema is not None:
            try:
                validate(instance=payload, schema=self.schema)
            except ValidationError as e:
                return ("invalid", e.message)

        offline_doc = self._sensor_document(payload, online=False)

        if not self.enabled or not self.firestore_online or self.sensor_col is None:
            queue_file = self.fallback.queue_firestore_add(
                FIREBASE_COLLECTION,
                offline_doc,
                "Firestore unavailable or not configured",
            )
            return ("queued", queue_file)

        try:
            doc_ref = self.sensor_col.add(self._sensor_document(payload, online=True))
            return ("ok", doc_ref[1].id)
        except Exception as e:
            queue_file = self.fallback.queue_firestore_add(
                FIREBASE_COLLECTION,
                offline_doc,
                f"Firestore write failed: {e}",
            )
            return ("queued", queue_file)

    def upload_wav(self, wav_path: Path, mac_compact: str):
        blob_path = f"audio/{mac_compact}/{wav_path.name}"

        if not self.storage_online or self.bucket is None:
            self.fallback.queue_storage_upload(
                wav_path,
                blob_path,
                content_type="audio/wav",
                bucket=FIREBASE_STORAGE_BUCKET,
                reason="Firebase Storage unavailable or not configured",
            )
            return blob_path, None

        try:
            blob = self.bucket.blob(blob_path)
            blob.upload_from_filename(str(wav_path), content_type="audio/wav")
            blob.make_public()
            print(f"[FIREBASE] audio uploaded: gs://{self.bucket.name}/{blob_path}")
            return blob_path, blob.public_url
        except Exception as e:
            print(f"[FIREBASE] audio upload failed: {e}")
            self.fallback.queue_storage_upload(
                wav_path,
                blob_path,
                content_type="audio/wav",
                bucket=getattr(self.bucket, "name", FIREBASE_STORAGE_BUCKET),
                reason=f"Firebase Storage upload failed: {e}",
            )
            return blob_path, None

    def publish_warning(self, doc: dict):
        offline_doc = self._warning_document(doc, online=False)

        if not self.enabled or not self.firestore_online or self.warnings_col is None:
            queue_file = self.fallback.queue_firestore_add(
                FIREBASE_WARNINGS_COLLECTION,
                offline_doc,
                "Firestore unavailable or not configured",
            )
            print(f"[FIREBASE] warning doc queued locally: {queue_file}")
            return "queued", queue_file

        try:
            doc_ref = self.warnings_col.add(self._warning_document(doc, online=True))
            doc_id = doc_ref[1].id
            print(f"[FIREBASE] warning doc written: {FIREBASE_WARNINGS_COLLECTION}/{doc_id}")
            return "ok", doc_id
        except Exception as e:
            print(f"[FIREBASE] warning write failed: {e}")
            queue_file = self.fallback.queue_firestore_add(
                FIREBASE_WARNINGS_COLLECTION,
                offline_doc,
                f"Firestore warning write failed: {e}",
            )
            print(f"[FIREBASE] warning doc queued locally: {queue_file}")
            return "queued", queue_file

    def publish_weather_reference(self, doc_id: str, doc: dict):
        if not self.enabled or not self.firestore_online or self.weather_col is None:
            print(f"[WEATHER] Firestore unavailable; skipping weather_reference/{doc_id}")
            return False
        try:
            payload = dict(doc)
            payload["received_at"] = firestore.SERVER_TIMESTAMP
            self.weather_col.document(doc_id).set(payload, merge=True)
            print(f"[WEATHER] {FIREBASE_WEATHER_COLLECTION}/{doc_id} updated: pressure={doc.get('pressure_hpa')}hPa")
            return True
        except Exception as e:
            print(f"[WEATHER] write failed for {doc_id}: {e}")
            return False


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


def normalize_pcm_16le(pcm_data: bytes, target_peak: int = NORMALIZE_TARGET_PEAK, max_gain: float = NORMALIZE_MAX_GAIN):
    """Peak-normalise signed 16-bit little-endian mono PCM without clipping.

    Returns (normalised_pcm, gain, original_peak).
    """
    if not pcm_data or len(pcm_data) < 2:
        return pcm_data, 1.0, 0

    sample_count = len(pcm_data) // 2
    pcm_data = pcm_data[:sample_count * 2]
    samples = struct.unpack("<" + "h" * sample_count, pcm_data)
    original_peak = max(abs(sample) for sample in samples)

    if original_peak == 0:
        return pcm_data, 1.0, 0

    target_peak = max(1, min(int(target_peak), 32767))
    gain = min(target_peak / original_peak, max_gain)

    # Do not reduce already-loud audio. This is mainly to lift quiet clips for transcription.
    if gain <= 1.0:
        return pcm_data, 1.0, original_peak

    normalised = [
        max(-32768, min(32767, int(round(sample * gain))))
        for sample in samples
    ]
    return struct.pack("<" + "h" * sample_count, *normalised), gain, original_peak


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
        elif status == "queued":
            print(f"[SENSOR]   -> queued locally for Firebase upload: {detail}")
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

            normalisation_gain = 1.0
            normalisation_peak = 0
            if NORMALIZE_AUDIO:
                pcm, normalisation_gain, normalisation_peak = normalize_pcm_16le(pcm)
                normalised_pcm_dump_path = session.session_dir / "decoded_pcm_normalised.raw"
                with open(normalised_pcm_dump_path, "wb") as f:
                    f.write(pcm)
                print(
                    f"[AUDIO] Normalised PCM: peak={normalisation_peak}, "
                    f"gain={normalisation_gain:.2f}x, target_peak={NORMALIZE_TARGET_PEAK}"
                )
            else:
                normalised_pcm_dump_path = None
                print("[AUDIO] PCM normalisation disabled")

            out_path = write_wav_file(
                node_mac=session.node_mac,
                sample_rate=session.sample_rate,
                total_samples=session.total_samples,
                pcm_data=pcm,
            )
            with open(session.info_path, "a", encoding="utf-8") as f:
                f.write(f"assembled_adpcm={assembled_adpcm_path}\n")
                f.write(f"decoded_pcm={pcm_dump_path}\n")
                if normalised_pcm_dump_path is not None:
                    f.write(f"decoded_pcm_normalised={normalised_pcm_dump_path}\n")
                f.write(f"normalise_audio={NORMALIZE_AUDIO}\n")
                f.write(f"normalisation_peak={normalisation_peak}\n")
                f.write(f"normalisation_gain={normalisation_gain:.4f}\n")
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
                    "audio_normalised": NORMALIZE_AUDIO,
                    "audio_normalisation_gain": normalisation_gain,
                    "audio_original_peak": normalisation_peak,
                    "audio_storage_path": storage_info[0] if storage_info else None,
                    "audio_download_url": storage_info[1] if storage_info else None,
                    "audio_local_path": str(out_path),
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
    print(f"Firebase offline queue will be saved to: {FIREBASE_FALLBACK_DIR}")

    monitor = UARTAudioMonitor(firebase_client)

    print("[WEATHER] fetching initial Auckland (Airport) pressure...")
    update_weather_reference(firebase_client)
    next_weather_fetch = time.time() + METSERVICE_FETCH_INTERVAL_S

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                monitor.buffer.extend(chunk)
                monitor.process_buffer()

            monitor.cleanup_stale_sessions()

            if time.time() >= next_weather_fetch:
                update_weather_reference(firebase_client)
                next_weather_fetch = time.time() + METSERVICE_FETCH_INTERVAL_S
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
