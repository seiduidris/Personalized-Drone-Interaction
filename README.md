# Personalized and Authorized Drone Operation with Facial Recognition and Gesture

## Description
This project presents a hand gesture control with facial authentication for the DJI Tello drone. The system uses computer vision (openCV) and machine learning techniques to ensure secure and intuitive drone control. Gestures work only if an authorized face is detected; otherwise, the drone lands after 20 seconds of not detecting any face.

## Features
- **Facial Authentication:** Utilizes facial recognition to authorize users.
- **Hand Gesture Control:** Controls drone movements based on recognized gestures.
  - Moves by 5 units once a face is recognized.
  - **Stop Gesture:** Stops the drone from moving.
  - **Thumbs Up Gesture:** Flips the drone (flip not recommended as it distorts video capturing).
  - **Peace Gesture:** Lands the drone.
- **Supported Gestures:**
  - okay
  - peace
  - thumbs up
  - thumbs down
  - call me
  - stop
  - rock
  - live long
  - fist
  - smile
- Customize movements by attaching gestures to desired commands.

## Setup Instructions
1. **Add Authorized Faces:**
   - Place images of people you want to authorize in the `Images` folder.
   
2. **Generate Encodings:**
   - Run the `EncodeGenerator.py` file to create the encoding file (`EncodeFile.p`).

3. **Run the Control Script:**
   - Execute the `real_adaptive_hand_gesture.py` script to start controlling the drone.

## Usage
- Ensure the drone is powered on and connected.
- Use authorized gestures to control the drone as specified.
- For real-time operation, the script must detect an authorized face continuously.

## Notes
- The `Thumbs Up` gesture is mapped to flip the drone. This is not recommended due to potential video capture distortion.
- Ensure stable lighting conditions for optimal performance of the facial recognition and gesture detection modules.

## Setup Instructions
1. **Install Necessary Libraries:**
   ```bash
   pip install cmake dlib opencv-python mediapipe tensorflow djitellopy face_recognition numpy
