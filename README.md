# tremor-duino
This project is an attempt to fully recreate the functionality of the Emma Watch for damping tremor impact for Parkinson's patients.

It utilizes hobbyist electronics hardware along with Edge Impulse machine learning to develop a customizable machine-learning model that optimizes haptic motor vibration frequencies as applied at the surface of a users wrist to minimize tremor impact.

Phase 1) Code to read accelerometer output from a Nano 33 BLE Sense V2 and output to Edge Impulse for Fourier Analysis of peak tremor frequency.
Phase 2) Code to apply a haptic motor with varying frequencies to mitigate tremor impact to output to Edge Impulse as training data for developing a Machine Learning model to fit vibrating motor frequency to tremor frequency to minimize tremor severity. The model from Edge Impulse is then deployed to the Nano 33 BLE Sense V2 for realtime wearable frequency adaptations to realtime tremor.

