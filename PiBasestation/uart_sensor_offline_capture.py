#!/usr/bin/env python3
"""Offline UART sensor packet capture.

Reads newline-delimited JSON sensor packets from the basestation ESP32 over
UART, validates them against the shared node-data schema when available, and
stores upload-ready records locally. No Firebase connection is attempted.

Output files:
- sensor_upload_queue.jsonl: append-only Firestore-style operations, one per
  valid sensor packet.
- sensor_offline_export.json: grouped JSON export for inspection or import
  tooling.
- raw_uart_capture.jsonl: append-only raw UART lines, including invalid JSON.

Type 'dump' + Enter to broadcast the same single-byte DUMP_REQUEST command used
by uart_sensor_dump.py. Do not run both scripts against the same serial port at
the same time.
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

try:
    from dotenv import load_dotenv
except ModuleNotFoundError:
    def load_dotenv():
        return False

try:
    from jsonschema import ValidationError, validate
except ModuleNotFoundError:
    ValidationError = None
    validate = None

load_dotenv()

UART_PORT = os.getenv("UART_PORT", "/dev/serial0")
BAUD_RATE = int(os.getenv("UART_BAUD_RATE", "115200"))
TIMEOUT = 0.1

FIREBASE_COLLECTION = os.getenv("FIREBASE_COLLECTION", "iot-data")

DEFAULT_SCHEMA_PATH = Path(__file__).resolve().parents[1] / "node-data-schema.json"
SCHEMA_PATH = Path(os.getenv("SCHEMA_PATH", str(DEFAULT_SCHEMA_PATH)))

FRAME_SYNC1 = 0xAA
FRAME_SYNC2 = 0x55
DUMP_REQUEST_MSG_TYPE = 0x06

DESKTOP_DIR = Path.home() / "Desktop"
DEFAULT_OUTPUT_DIR = (
    DESKTOP_DIR if DESKTOP_DIR.exists() else Path.home()
) / "uart_sensor_offline_capture"


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
    """Ask for a local base time and return (base_datetime, monotonic_start)."""
    local_tz = datetime.now().astimezone().tzinfo
    print(
        "Set the 'received_at' base time "
        f"(local: {local_tz}). Each packet will be stamped as base + elapsed."
    )
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
    print(
        f"[TIME] base = {base.strftime('%Y-%m-%d %H:%M:%S %Z')} "
        f"({base.astimezone(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')})"
    )
    return base, monotonic_start


def load_schema(path: Path):
    if validate is None:
        print("[SCHEMA] jsonschema is not installed; validation disabled")
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            schema = json.load(f)
        print(f"[SCHEMA] loaded from {path}")
        return schema
    except Exception as e:
        print(f"[SCHEMA] not loaded from {path}: {e}")
        return None


class OfflineCaptureStore:
    """Append-only local storage for later upload."""

    def __init__(self, output_dir: Path, collection: str):
        self.output_dir = output_dir
        self.collection = collection
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.queue_path = self.output_dir / "sensor_upload_queue.jsonl"
        self.raw_path = self.output_dir / "raw_uart_capture.jsonl"
        self.export_path = self.output_dir / "sensor_offline_export.json"
        self.readme_path = self.output_dir / "README.txt"
        self._write_readme()
        self._refresh_export()

    def _write_readme(self):
        if self.readme_path.exists():
            return
        self.readme_path.write_text(
            "UART sensor offline capture\n"
            "===========================\n\n"
            "Files:\n"
            "- sensor_upload_queue.jsonl: append-only upload queue. Each line is a JSON "
            "operation with operation='firestore_add', collection, document, and queued_at.\n"
            "- sensor_offline_export.json: grouped JSON view of the upload queue.\n"
            "- raw_uart_capture.jsonl: raw UART text lines, including non-JSON or invalid "
            "packets, for debugging.\n\n"
            "After uploading/importing the data, archive or delete these files to avoid "
            "duplicate uploads.\n",
            encoding="utf-8",
        )

    def _append_jsonl(self, path: Path, record: dict):
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _read_upload_queue(self) -> list:
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
                    print(f"[STORE] skipped invalid upload queue line {line_no}: {e}")
        return records

    def _refresh_export(self):
        collections = {}
        for record in self._read_upload_queue():
            if record.get("operation") == "firestore_add":
                collection = record.get("collection", "unknown")
                collections.setdefault(collection, []).append(record.get("document", {}))

        export = {
            "generated_at": utc_now_iso(),
            "queue_file": str(self.queue_path),
            "raw_capture_file": str(self.raw_path),
            "collections": collections,
        }
        with open(self.export_path, "w", encoding="utf-8") as f:
            json.dump(export, f, indent=2, ensure_ascii=False)

    def record_raw_line(self, text: str, status: str, detail: str | None = None):
        record = {
            "captured_at": utc_now_iso(),
            "status": status,
            "line": text,
        }
        if detail:
            record["detail"] = detail
        self._append_jsonl(self.raw_path, record)

    def queue_sensor_document(self, document: dict) -> str:
        record = {
            "operation": "firestore_add",
            "collection": self.collection,
            "document": document,
            "queued_at": utc_now_iso(),
            "source": "uart_sensor_offline_capture.py",
        }
        self._append_jsonl(self.queue_path, record)
        self._refresh_export()
        return str(self.queue_path)


class SensorOfflineMonitor:
    def __init__(
        self,
        store: OfflineCaptureStore,
        base_time: datetime,
        monotonic_start: float,
        schema: dict | None,
        serial_port=None,
    ):
        self.buffer = bytearray()
        self.store = store
        self.base_time = base_time
        self.monotonic_start = monotonic_start
        self.schema = schema
        self.serial_port = serial_port
        self.saved_count = 0
        self.invalid_count = 0

    def _current_received_at(self) -> datetime:
        elapsed = time.monotonic() - self.monotonic_start
        return self.base_time + timedelta(seconds=elapsed)

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

    def _sensor_document(self, payload: dict) -> dict:
        received_at = self._current_received_at()
        return {
            **payload,
            "received_at": received_at.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "received_by": "pi-basestation",
            "offline_captured": True,
        }

    def handle_text_line(self, text: str):
        try:
            payload = json.loads(text)
        except json.JSONDecodeError as e:
            self.invalid_count += 1
            self.store.record_raw_line(text, "invalid_json", str(e))
            print(text)
            return

        if not isinstance(payload, dict):
            self.invalid_count += 1
            self.store.record_raw_line(text, "non_object_json")
            print(text)
            return

        if self.schema is not None:
            try:
                validate(instance=payload, schema=self.schema)
            except ValidationError as e:
                self.invalid_count += 1
                self.store.record_raw_line(text, "schema_invalid", e.message)
                print(f"[SENSOR] schema validation failed: {e.message}")
                return

        self.store.record_raw_line(text, "queued")
        document = self._sensor_document(payload)
        queue_file = self.store.queue_sensor_document(document)
        self.saved_count += 1

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
        print(f"[SENSOR]   -> queued offline ({self.saved_count} saved): {queue_file}")

    def process_buffer(self):
        while self.buffer:
            newline_index = self.buffer.find(b"\n")
            if newline_index == -1:
                if len(self.buffer) > 4096:
                    text = safe_text_from_bytes(bytes(self.buffer))
                    if text:
                        self.handle_text_line(text)
                    self.buffer.clear()
                return
            line = bytes(self.buffer[: newline_index + 1])
            del self.buffer[: newline_index + 1]
            text = safe_text_from_bytes(line)
            if text:
                self.handle_text_line(text)


def stdin_reader(cmd_queue: "queue.Queue[str]"):
    for line in sys.stdin:
        cmd_queue.put(line.strip().lower())
    cmd_queue.put("__eof__")


def main():
    parser = argparse.ArgumentParser(
        description="Capture UART sensor packets to upload-ready offline files."
    )
    parser.add_argument("--port", default=UART_PORT, help=f"serial port (default: {UART_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help=f"baud rate (default: {BAUD_RATE})")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(os.getenv("UART_OFFLINE_CAPTURE_DIR", str(DEFAULT_OUTPUT_DIR))),
        help=f"output directory (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--collection",
        default=FIREBASE_COLLECTION,
        help=f"target Firestore collection stored in queue records (default: {FIREBASE_COLLECTION})",
    )
    parser.add_argument(
        "--no-schema",
        action="store_true",
        help="store JSON object packets without validating against node-data-schema.json",
    )
    parser.add_argument(
        "--no-initial-dump",
        action="store_true",
        help="do not send DUMP_REQUEST immediately after opening the serial port",
    )
    args = parser.parse_args()

    try:
        import serial
    except ModuleNotFoundError:
        print("Missing dependency: pyserial. Install it with: pip install pyserial")
        return

    base_time, monotonic_start = prompt_base_time()
    schema = None if args.no_schema else load_schema(SCHEMA_PATH)
    store = OfflineCaptureStore(args.out_dir, args.collection)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"Failed to open serial port {args.port}: {e}")
        return

    monitor = SensorOfflineMonitor(store, base_time, monotonic_start, schema, serial_port=ser)

    if not args.no_initial_dump:
        monitor.send_dump_request()

    print(f"Listening on {args.port} at {args.baud} baud...")
    print(f"Upload queue: {store.queue_path}")
    print(f"Grouped export: {store.export_path}")
    print(f"Raw UART capture: {store.raw_path}")
    print("Type 'dump' + Enter to broadcast another audio dump request. 'quit' or Ctrl-C to exit.")

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
                        pass
                    elif cmd:
                        print(f"[CMD] unknown command '{cmd}' (try 'dump' or 'quit')")
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        print(
            f"\nStopping. Saved {monitor.saved_count} packet(s); "
            f"ignored {monitor.invalid_count} invalid line(s)."
        )
    finally:
        ser.close()


if __name__ == "__main__":
    main()
