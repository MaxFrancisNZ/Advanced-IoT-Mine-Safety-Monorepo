import os
import json
import serial
import firebase_admin
from firebase_admin import credentials, firestore
from jsonschema import validate, ValidationError
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Serial port settings
SERIAL_PORT = os.getenv('SERIAL_PORT', '/dev/ttyUSB0')
SERIAL_BAUD_RATE = int(os.getenv('SERIAL_BAUD_RATE', '115200'))

# Firebase settings
FIREBASE_CREDENTIALS_PATH = os.getenv('FIREBASE_CREDENTIALS_PATH')
FIREBASE_COLLECTION = os.getenv('FIREBASE_COLLECTION', 'iot-data')

# Schema path
SCHEMA_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'node-data-schema.json')

def load_schema():
    """Load the JSON schema."""
    try:
        with open(SCHEMA_PATH, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f'Could not load schema from {SCHEMA_PATH}: {e}')
        return None

def main():
    if not FIREBASE_CREDENTIALS_PATH:
        raise ValueError('FIREBASE_CREDENTIALS_PATH environment variable is required')

    # Initialize Firebase
    cred = credentials.Certificate(FIREBASE_CREDENTIALS_PATH)
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    collection_ref = db.collection(FIREBASE_COLLECTION)

    schema = load_schema()

    # Open serial port
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD_RATE, timeout=1)
    print(f'Serial port opened: {SERIAL_PORT} @ {SERIAL_BAUD_RATE}')

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue

            try:
                payload = json.loads(line)
            except json.JSONDecodeError as e:
                print(f'Received invalid JSON over serial: {line}')
                print(f'Error: {e}')
                continue

            if schema:
                try:
                    validate(instance=payload, schema=schema)
                except ValidationError as e:
                    print(f'Payload failed schema validation: {e.message}')
                    print(f'Raw payload: {json.dumps(payload)}')
                    continue

            try:
                doc_ref = collection_ref.add({
                    **payload,
                    'received_at': firestore.SERVER_TIMESTAMP,
                    'received_by': 'pi-basestation'
                })
                print(f'Document written to Firestore collection \'{FIREBASE_COLLECTION}\' with ID: {doc_ref[1].id}')
            except Exception as e:
                print(f'Failed to publish payload to Firestore: {e}')

    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        ser.close()

if __name__ == '__main__':
    main()