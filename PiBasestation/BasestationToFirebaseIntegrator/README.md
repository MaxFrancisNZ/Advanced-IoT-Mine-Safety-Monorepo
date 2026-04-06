# BasestationToFirebaseIntegrator (Python Version)

This Python service runs on the Raspberry Pi and listens to the serial port connected to the ESP32 basestation receiver.
When the ESP32 forwards an ESP-NOW payload over UART, this integrator parses the incoming JSON and writes it to a Firestore collection.

## Requirements

- Python 3.13+

## Setup

1. Install dependencies:

   pip install -r requirements.txt

2. Copy `.env.example` to `.env` and fill in the Firebase credentials and serial port settings.

3. Run the integrator:

   python integrator.py

## Configuration

- `SERIAL_PORT` — The serial device path for the ESP32 receiver.
- `SERIAL_BAUD_RATE` — The baud rate used by the ESP32 UART bridge.
- `FIREBASE_CREDENTIALS_PATH` — Path to Firebase service account key JSON.
- `FIREBASE_COLLECTION` — Firestore collection name.

## Notes

- The ESP32 basestation firmware currently forwards raw JSON payloads from incoming ESP-NOW messages over UART.
- The integrator reads each newline-delimited JSON payload and writes it to Firestore with a `received_at` timestamp.
