const { initializeApp } = require('firebase/app');
const { getFirestore, collection, addDoc } = require('firebase/firestore');

// Firebase configuration (replace with your own or use environment variables)
const firebaseConfig = {
  apiKey: process.env.FIREBASE_API_KEY || "your-api-key",
  authDomain: process.env.FIREBASE_AUTH_DOMAIN || "your-project.firebaseapp.com",
  projectId: process.env.FIREBASE_PROJECT_ID || "your-project-id",
  storageBucket: process.env.FIREBASE_STORAGE_BUCKET || "your-project.appspot.com",
  messagingSenderId: process.env.FIREBASE_MESSAGING_SENDER_ID || "123456789",
  appId: process.env.FIREBASE_APP_ID || "1:123456789:web:abcdef"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const db = getFirestore(app);

// Function to generate stub data based on the schema
function generateStubData() {
  const nodeId = generateMacAddress();
  const transmissionAttempts = Math.floor(Math.random() * 11); // 0-10
  const temperature = 20 + Math.random() * 10; // 20-30
  const humidity = 40 + Math.random() * 20; // 40-60
  const accelerometer = {
    x: (Math.random() - 0.5) * 2, // -1 to 1
    y: (Math.random() - 0.5) * 2,
    z: (Math.random() - 0.5) * 2
  };
  const gyroscope = {
    x: (Math.random() - 0.5) * 2,
    y: (Math.random() - 0.5) * 2,
    z: (Math.random() - 0.5) * 2
  };
  const barometricPressure = 1000 + Math.random() * 20; // 1000-1020
  const ledState = generateHexColor();

  return {
    node_id: nodeId,
    transmission_attempts: transmissionAttempts,
    environment: {
      temperature: parseFloat(temperature.toFixed(2)),
      humidity: parseFloat(humidity.toFixed(2)),
      accelerometer,
      gyroscope,
      barometric_pressure: parseFloat(barometricPressure.toFixed(2))
    },
    led_state: ledState
  };
}

// Helper function to generate a fake MAC address
function generateMacAddress() {
  const hexDigits = '0123456789ABCDEF';
  let mac = '';
  for (let i = 0; i < 6; i++) {
    mac += hexDigits.charAt(Math.floor(Math.random() * 16));
    mac += hexDigits.charAt(Math.floor(Math.random() * 16));
    if (i < 5) mac += ':';
  }
  return mac;
}

// Helper function to generate a random hex color
function generateHexColor() {
  const hexDigits = '0123456789ABCDEF';
  let color = '#';
  for (let i = 0; i < 6; i++) {
    color += hexDigits.charAt(Math.floor(Math.random() * 16));
  }
  return color;
}

// Function to publish data to Firebase
async function publishToFirebase(data, collectionName = 'iot-data') {
  try {
    const docRef = await addDoc(collection(db, collectionName), data);
    console.log('Document written with ID: ', docRef.id);
  } catch (e) {
    console.error('Error adding document: ', e);
  }
}

// Main function
async function main() {
  const collectionName = process.argv[2] || 'iot-data'; // Allow specifying collection via command line
  const stubData = generateStubData();
  console.log('Generated stub data:', JSON.stringify(stubData, null, 2));
  await publishToFirebase(stubData, collectionName);
}

main();