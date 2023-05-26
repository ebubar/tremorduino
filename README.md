# tremor-duino
This project is an attempt to fully recreate the functionality of the Emma Watch for damping tremor impact for Parkinson's patients.

It utilizes hobbyist electronics hardware along with Edge Impulse machine learning to develop a customizable machine-learning model that optimizes haptic motor vibration frequencies as applied at the surface of a users wrist to minimize tremor impact.

Phase 1) Code to read accelerometer output from a Nano 33 BLE Sense V2 and vibrate a small coin-cell motor and output data to Edge Impulse for Fourier Analysis of peak tremor frequency. 

Phase 2) Train a machine-learning model from Edge Impulse to deploy to the Nano 33 BLE Sense V2 for realtime wearable frequency adaptations to realtime tremor. Deploy this as an Arduino library.

ei-tremorduino-arduino-1.0.5.zip



