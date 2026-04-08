# StubForFirebase (Python Version)

This is a Python script that generates mock IoT data based on the `node-data-schema.json` and publishes it to a Firestore collection.

## Requirements

- Python 3.13+

## Setup

1. Install dependencies:

   pip install -r requirements.txt

2. Set up Firebase credentials:
   - Download your Firebase service account key JSON file.
   - Copy `.env.example` to `.env` and update `FIREBASE_CREDENTIALS_PATH` to point to your key file.

3. Run the script:

   python stub_publisher.py [collection_name]

   If no collection name is provided, it defaults to 'iot-data'.

## Notes

- Uses `firebase-admin` for Firestore access.
- Generates random data conforming to the schema.