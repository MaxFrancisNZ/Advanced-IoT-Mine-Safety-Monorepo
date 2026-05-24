#!/usr/bin/env python3
"""Sensor-only UART receiver with manual audio-dump trigger.

Companion to uart_audio.py. Reads JSON sensor lines from the basestation ESP32
over UART, validates them against the shared node-data schema, and pushes them
to Firebase (with the same offline-queue fallback as uart_audio.py). Audio
handling (ADPCM decode, Whisper transcription, GPT classification, WAV upload)
is intentionally omitted.

Provides one manual command: type 'dump' + Enter at the terminal to broadcast a
single-byte DUMP_REQUEST (0x06) command frame back through the Pi -> ESP32 ->
ESP-NOW chain. The wearable firmware treats this as an "audio dump" trigger.

Do NOT run this script and uart_audio.py against the same serial port at the
same time -- they will fight for reads.
"""

import argparse
import json
import os
import queue
import sys
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import serial
import firebase_admin
from firebase_admin import credentials, firestore
from jsonschema import validate, ValidationError
from dotenv import load_dotenv

load_dotenv()

UART_PORT = os.getenv("UART_PORT", "/dev/serial0")
BAUD_RATE = int(os.getenv("UART_BAUD_RATE", "115200"))
TIMEOUT = 0.1

# --- Firebase config ---
FIREBASE_CREDENTIALS_PATH = os.getenv("FIREBASE_CREDENTIALS_PATH")
FIREBASE_COLLECTION = os.getenv("FIREBASE_COLLECTION", "iot-data")

DEFAULT_SCHEMA_PATH = (
    Path(__file__).resolve().parent
    / "Advanced-IoT-Mine-Safety-Monorepo"
    / "node-data-schema.json"
)
SCHEMA_PATH = Path(os.getenv("SCHEMA_PATH", str(DEFAULT_SCHEMA_PATH)))

# Pi -> wearable command frame: [0xAA 0x55 len_lo len_hi payload crc8]
FRAME_SYNC1 = 0xAA
FRAME_SYNC2 = 0x55

# Dump-request opcode: matches DUMP_REQUEST_MSG_TYPE in
# ESPWearableTransceiver/main/audio_protocol.h. The wearable broadcasts this
# request to all peers and dispatches purely on the opcode byte -- no MAC,
# no session ID required.
DUMP_REQUEST_MSG_TYPE = 0x06

DESKTOP_DIR = Path.home() / "Desktop"
OUTPUT_DIR = DESKTOP_DIR if DESKTOP_DIR.exists() else Path.home()
FIREBASE_FALLBACK_DIR = Path(
    os.getenv("FIREBASE_FALLBACK_DIR", str(OUTPUT_DIR / "firebase_offline_queue"))
)


def utc_now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _crc8_07(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_command_frame(payload: bytes) -> bytes:
    """Wrap a wearable-bound command in [0xAA 0x55 len_lo len_hi payload crc8]."""
    if len(payload) == 0 or len(payload) > 0xFFFF:
        raise ValueError(f"command payload length out of range: {len(payload)}")
    length = len(payload)
    header = bytes([length & 0xFF, (length >> 8) & 0xFF])
    crc = _crc8_07(header + payload)
    return bytes([FRAME_SYNC1, FRAME_SYNC2]) + header + payload + bytes([crc])


def safe_text_from_bytes(data: bytes) -> str:
    return data.decode("utf-8", errors="replace").rstrip("\r\n")


def prompt_int(label: str, min_val: int, max_val: int) -> int:
    while True:
        raw = input(f"Enter {label}: ").strip()
        try:
            val = int(raw)
        except ValueError:
            print(f"  invalid: '{raw}' is not an integer")
            continue
        if not (min_val <= val <= max_val):
            print(f"  invalid: {val} outside [{min_val}, {max_val}]")
            continue
        return val


def prompt_base_time():
    """Ask the user for a local date/time and return (base_datetime, monotonic_start).

    The returned datetime is tz-aware (system local zone). Each subsequent packet is
    timestamped as base_datetime + (now - monotonic_start), so the user-supplied time
    acts as a fixed offset against the system monotonic clock.
    """
    local_tz = datetime.now().astimezone().tzinfo
    print(f"Set the 'received_at' base time (local: {local_tz}). Each packet will be stamped as base + elapsed.")
    while True:
        year = prompt_int("Year (yyyy)", 2000, 2100)
        month = prompt_int("Month (1-12)", 1, 12)
        day = prompt_int("Day (1-31)", 1, 31)
        hour = prompt_int("Hour (0-23)", 0, 23)
        minute = prompt_int("Minute (0-59)", 0, 59)
        second = prompt_int("Second (0-59)", 0, 59)
        try:
            base = datetime(year, month, day, hour, minute, second, tzinfo=local_tz)
            break
        except ValueError as e:
            print(f"  invalid date/time: {e} -- try again")
    monotonic_start = time.monotonic()
    print(f"[TIME] base = {base.strftime('%Y-%m-%d %H:%M:%S %Z')} "
          f"({base.astimezone(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')})")
    return base, monotonic_start


class FirebaseFallbackStore:
    """Local append-only queue for Firestore writes that could not be sent."""

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
            "or when an individual Firestore operation fails.\n\n"
            "Files:\n"
            "- firebase_upload_queue.jsonl: append-only queue; one JSON operation per line.\n"
            "- firestore_offline_export.json: easier-to-read export grouped by Firestore collection.\n\n"
            "Operation types:\n"
            "- firestore_add: add the supplied document to the named Firestore collection.\n\n"
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
        for record in records:
            if record.get("operation") == "firestore_add":
                collections.setdefault(record.get("collection", "unknown"), []).append(record.get("document", {}))

        export = {
            "generated_at": utc_now_iso(),
            "queue_file": str(self.queue_path),
            "collections": collections,
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


class FirebaseClient:
    def __init__(self, base_time: datetime, monotonic_start: float):
        self.base_time = base_time
        self.monotonic_start = monotonic_start
        self.enabled = False
        self.firestore_online = False
        self.sensor_col = None
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
            firebase_admin.initialize_app(cred)

            db = firestore.client()
            self.sensor_col = db.collection(FIREBASE_COLLECTION)

            self._load_schema()

            print(f"[FIREBASE] initialized (sensor='{FIREBASE_COLLECTION}')")
            self.firestore_online = self._test_connection()
            self.enabled = self.firestore_online
            if not self.firestore_online:
                print("[FIREBASE] Firestore unavailable; new records will be written to the offline queue")
        except Exception as e:
            print(f"[FIREBASE] disabled: init failed: {e}")
            print("[FIREBASE] new Firebase records will be written to the offline queue")
            self.enabled = False
            self.firestore_online = False

    def _load_schema(self):
        try:
            with open(SCHEMA_PATH, "r") as f:
                self.schema = json.load(f)
            print(f"[FIREBASE] schema loaded from {SCHEMA_PATH}")
        except Exception as e:
            print(f"[FIREBASE] schema not loaded from {SCHEMA_PATH}: {e}")
            self.schema = None

    def _test_connection(self) -> bool:
        try:
            ping_ref = self.sensor_col.document("_connection_test")
            ping_ref.set({
                "ping": True,
                "tested_at": firestore.SERVER_TIMESTAMP,
                "tested_by": "pi-basestation",
            })
            ping_ref.delete()
            print(f"[FIREBASE] [OK] Firestore reachable (write+delete to {FIREBASE_COLLECTION}/_connection_test)")
            return True
        except Exception as e:
            print(f"[FIREBASE] [FAIL] Firestore test failed: {e}")
            print("[FIREBASE] (check service account project, IAM roles, and that Firestore is enabled)")
            return False

    def _current_received_at(self) -> datetime:
        elapsed = time.monotonic() - self.monotonic_start
        return self.base_time + timedelta(seconds=elapsed)

    def _sensor_document(self, payload: dict, received_at: datetime, online: bool) -> dict:
        if online:
            return {
                **payload,
                "received_at": received_at,
                "received_by": "pi-basestation",
            }
        return {
            **payload,
            "received_at": received_at.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "received_by": "pi-basestation",
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

        received_at = self._current_received_at()
        offline_doc = self._sensor_document(payload, received_at, online=False)

        if not self.enabled or not self.firestore_online or self.sensor_col is None:
            queue_file = self.fallback.queue_firestore_add(
                FIREBASE_COLLECTION,
                offline_doc,
                "Firestore unavailable or not configured",
            )
            return ("queued", queue_file)

        try:
            doc_ref = self.sensor_col.add(self._sensor_document(payload, received_at, online=True))
            return ("ok", doc_ref[1].id)
        except Exception as e:
            queue_file = self.fallback.queue_firestore_add(
                FIREBASE_COLLECTION,
                offline_doc,
                f"Firestore write failed: {e}",
            )
            return ("queued", queue_file)


class DelayedUploader:
    """Buffer parsed sensor payloads and trickle them to Firebase one-per-interval.

    Packets are sent in arrival order (FIFO). The worker sleeps `interval` seconds
    after each send, so a burst of N packets takes ~N*interval to drain.
    `publish_sensor_payload` is invoked at send time, so `received_at` reflects
    the upload moment rather than the arrival moment -- intentional, so Firestore
    sees evenly-paced data.
    """

    def __init__(self, firebase: FirebaseClient, interval: float):
        self.firebase = firebase
        self.interval = interval
        self.queue: "queue.Queue[dict]" = queue.Queue()
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def submit(self, payload: dict) -> int:
        self.queue.put(payload)
        return self.queue.qsize()

    def _worker(self):
        while True:
            payload = self.queue.get()
            status, detail = self.firebase.publish_sensor_payload(payload)
            depth = self.queue.qsize()
            if status == "ok":
                print(f"[DELAY] -> Firestore OK: {FIREBASE_COLLECTION}/{detail} (remaining: {depth})")
            elif status == "queued":
                print(f"[DELAY] -> queued locally for Firebase upload: {detail} (remaining: {depth})")
            elif status == "invalid":
                print(f"[DELAY] -> schema validation failed: {detail} (remaining: {depth})")
            elif status == "error":
                print(f"[DELAY] -> Firestore write FAILED: {detail} (remaining: {depth})")
            time.sleep(self.interval)


class SensorMonitor:
    def __init__(self, firebase: FirebaseClient, serial_port=None, uploader: "DelayedUploader | None" = None):
        self.buffer = bytearray()
        self.firebase = firebase
        self.serial_port = serial_port
        self.uploader = uploader

    def _send_command_frame(self, payload: bytes, log_label: str):
        if self.serial_port is None:
            print(f"[CMD] no serial port; cannot send {log_label}")
            return
        try:
            frame = build_command_frame(payload)
            self.serial_port.write(frame)
            print(f"[CMD] sent {log_label} ({len(frame)} bytes incl. framing)")
        except Exception as e:
            print(f"[CMD] failed to send {log_label}: {e}")

    def send_dump_request(self):
        self._send_command_frame(
            bytes([DUMP_REQUEST_MSG_TYPE]),
            f"DUMP_REQUEST (0x{DUMP_REQUEST_MSG_TYPE:02X}) broadcast",
        )

    def handle_text_line(self, text: str):
        """Pretty-print a UART text line. If it's a JSON sensor payload, also submit it."""
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
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

        if self.uploader is not None:
            depth = self.uploader.submit(payload)
            print(f"[SENSOR]   -> buffered for delayed upload (depth: {depth})")
            return

        status, detail = self.firebase.publish_sensor_payload(payload)
        if status == "ok":
            print(f"[SENSOR]   -> Firestore OK: {FIREBASE_COLLECTION}/{detail}")
        elif status == "queued":
            print(f"[SENSOR]   -> queued locally for Firebase upload: {detail}")
        elif status == "invalid":
            print(f"[SENSOR]   -> schema validation failed: {detail}")
        elif status == "error":
            print(f"[SENSOR]   -> Firestore write FAILED: {detail}")

    def process_buffer(self):
        while self.buffer:
            newline_index = self.buffer.find(b"\n")
            if newline_index == -1:
                if len(self.buffer) > 4096:
                    text = safe_text_from_bytes(bytes(self.buffer))
                    if text:
                        print(text)
                    self.buffer.clear()
                return
            line = bytes(self.buffer[:newline_index + 1])
            del self.buffer[:newline_index + 1]
            text = safe_text_from_bytes(line)
            if text:
                self.handle_text_line(text)


def stdin_reader(cmd_queue: "queue.Queue[str]"):
    """Daemon thread: forward each stdin line (lowercased, stripped) to the queue."""
    for line in sys.stdin:
        cmd_queue.put(line.strip().lower())
    cmd_queue.put("__eof__")


def main():
    parser = argparse.ArgumentParser(
        description="Sensor-only UART receiver with a manual audio-dump trigger."
    )
    parser.add_argument("--port", default=UART_PORT, help=f"serial port (default: {UART_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help=f"baud rate (default: {BAUD_RATE})")
    parser.add_argument(
        "--no-proxy",
        action="store_true",
        help="Strip HTTP(S)_PROXY/ALL_PROXY from env before Firebase init. "
             "Needed when the network proxy blocks gRPC (Firestore will hang 60s and fail otherwise).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="Buffer received packets and upload them in FIFO order one per SECONDS. "
             "Default 0 uploads each packet to Firebase as soon as it arrives.",
    )
    args = parser.parse_args()

    if args.interval < 0:
        parser.error("--interval must be >= 0")

    if args.no_proxy:
        for var in ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY",
                    "http_proxy", "https_proxy", "all_proxy",
                    "GRPC_PROXY", "grpc_proxy"):
            os.environ.pop(var, None)
        print("[NET] proxy env vars stripped; gRPC/Firestore will connect directly.")

    base_time, monotonic_start = prompt_base_time()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"Failed to open serial port {args.port}: {e}")
        return

    # Send the dump immediately so it isn't delayed by Firebase init (which can hang
    # 60s on a proxied network). Build and write the frame directly -- no monitor or
    # firebase client needed for this single byte.
    try:
        dump_frame = build_command_frame(bytes([DUMP_REQUEST_MSG_TYPE]))
        ser.write(dump_frame)
        print(f"[CMD] sent DUMP_REQUEST (0x{DUMP_REQUEST_MSG_TYPE:02X}) broadcast "
              f"({len(dump_frame)} bytes incl. framing)")
    except Exception as e:
        print(f"[CMD] failed to send initial DUMP_REQUEST: {e}")

    firebase_client = FirebaseClient(base_time, monotonic_start)

    print(f"Listening on {args.port} at {args.baud} baud...")
    print(f"Firebase offline queue: {FIREBASE_FALLBACK_DIR}")
    print("Type 'dump' + Enter to broadcast another audio dump request. 'quit' or Ctrl-C to exit.")

    uploader = None
    if args.interval > 0:
        uploader = DelayedUploader(firebase_client, args.interval)
        print(f"[DELAY] upload interval: {args.interval}s "
              f"-- packets will be buffered in memory and sent to Firebase in arrival order")

    monitor = SensorMonitor(firebase_client, serial_port=ser, uploader=uploader)

    cmd_queue: "queue.Queue[str]" = queue.Queue()
    threading.Thread(target=stdin_reader, args=(cmd_queue,), daemon=True).start()

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                monitor.buffer.extend(chunk)
                monitor.process_buffer()

            try:
                while True:
                    cmd = cmd_queue.get_nowait()
                    if cmd in ("dump", "d"):
                        monitor.send_dump_request()
                    elif cmd in ("quit", "q", "exit"):
                        raise KeyboardInterrupt
                    elif cmd == "__eof__":
                        # stdin closed (e.g. running under a non-interactive shell);
                        # keep running on serial traffic alone.
                        pass
                    elif cmd:
                        print(f"[CMD] unknown command '{cmd}' (try 'dump' or 'quit')")
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
