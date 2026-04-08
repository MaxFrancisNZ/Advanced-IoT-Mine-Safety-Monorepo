import os
import json
import random
import firebase_admin
from firebase_admin import credentials, firestore

# Firebase configuration (use environment variables or service account key)
FIREBASE_CREDENTIALS_PATH = os.getenv('FIREBASE_CREDENTIALS_PATH', 'serviceAccountKey.json')
FIREBASE_COLLECTION = os.getenv('FIREBASE_COLLECTION', 'iot-data')

# Initialize Firebase
cred = credentials.Certificate(FIREBASE_CREDENTIALS_PATH)
firebase_admin.initialize_app(cred)
db = firestore.client()

def generate_mac_address():
    """Generate a fake MAC address."""
    return ':'.join(['{:02X}'.format(random.randint(0, 255)) for _ in range(6)])

def generate_hex_color():
    """Generate a random hex color."""
    return '#{:06X}'.format(random.randint(0, 0xFFFFFF))

def generate_stub_data():
    """Generate stub data based on the schema."""
    node_id = generate_mac_address()
    transmission_attempts = random.randint(0, 10)
    temperature = 20 + random.random() * 10  # 20-30
    humidity = 40 + random.random() * 20  # 40-60
    accelerometer = {
        'x': (random.random() - 0.5) * 2,
        'y': (random.random() - 0.5) * 2,
        'z': (random.random() - 0.5) * 2
    }
    gyroscope = {
        'x': (random.random() - 0.5) * 2,
        'y': (random.random() - 0.5) * 2,
        'z': (random.random() - 0.5) * 2
    }
    barometric_pressure = 1000 + random.random() * 20  # 1000-1020
    battery_voltage = round(3.0 + random.random() * 1.5, 2) if random.random() > 0.5 else None  # Optional, 3.0-4.5V or None
    led_state = generate_hex_color()

    data = {
        'node_id': node_id,
        'transmission_attempts': transmission_attempts,
        'environment': {
            'temperature': round(temperature, 2),
            'humidity': round(humidity, 2),
            'accelerometer': accelerometer,
            'gyroscope': gyroscope,
            'barometric_pressure': round(barometric_pressure, 2)
        },
        'led_state': led_state
    }
    if battery_voltage is not None:
        data['battery_voltage'] = battery_voltage
    return data

def publish_to_firebase(data, collection_name=FIREBASE_COLLECTION):
    """Publish data to Firebase Firestore."""
    try:
        doc_ref = db.collection(collection_name).add(data)
        print(f'Document written with ID: {doc_ref[1].id}')
    except Exception as e:
        print(f'Error adding document: {e}')

def main():
    collection_name = os.sys.argv[1] if len(os.sys.argv) > 1 else FIREBASE_COLLECTION
    stub_data = generate_stub_data()
    print('Generated stub data:', json.dumps(stub_data, indent=2))
    publish_to_firebase(stub_data, collection_name)

if __name__ == '__main__':
    main()