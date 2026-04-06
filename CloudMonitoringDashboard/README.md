# IoT Mine Safety Dashboard

A simple web dashboard to visualize IoT data from Firestore.

## Setup

1. Update `index.html` with your Firebase config (replace the placeholder values).
2. For local development: `npm install && npm start` (opens on http://localhost:3000)
3. For production: Deploy to Firebase Hosting (free) or any static host like Vercel/Netlify.

## Firebase Hosting Deployment (Recommended)

1. Install Firebase CLI: `npm install -g firebase-tools`
2. Login: `firebase login`
3. Init hosting: `cd Dashboard && firebase init hosting` (select your project)
4. Deploy: `firebase deploy --only hosting`
5. Access at: `https://your-project.web.app`

## Features

- Real-time updates from Firestore
- Charts for temperature and humidity
- Table of recent data points
- Responsive design

## Security Note

This dashboard reads from Firestore publicly. In production, add Firestore security rules to restrict access.