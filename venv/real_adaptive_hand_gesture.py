import time
import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model
from djitellopy import Tello
import face_recognition
import pickle

# Create Tello object
tello = Tello()

# Initialize mediapipe
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mpDraw = mp.solutions.drawing_utils

# Load the gesture recognizer model
model = load_model('hand-gesture-recognition-code/mp_hand_gesture')

# Load class names for hand gestures
with open('hand-gesture-recognition-code/gesture.names', 'r') as f:
    classNames = f.read().split('\n')

# Load the encoding file
print("Loading Encode File ...")
file = open("EncodeFile.p", 'rb')
encodeListKnownWithIds = pickle.load(file)
file.close()
encodeListKnown, studentIds = encodeListKnownWithIds
print("Encode File loaded")

# Connect to the drone
tello.connect()
time.sleep(1)

# Send the takeoff command
tello.takeoff()
time.sleep(1)
tello.move_up(20)
time.sleep(2)

# Start video streaming
tello.stream_on()

# Debouncing settings
debounce_frame_count = 30   
gesture_buffer = []
current_gesture = ''
confirmed_gesture = ''
last_face_detected_time = time.time()

try:
    while True:
        img = tello.get_frame()
        if img is not None:
            framergb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # Face recognition
            imgS = cv2.resize(framergb, (0, 0), None, 0.25, 0.25)
            faceCurFrame = face_recognition.face_locations(imgS)
            encodeCurFrame = face_recognition.face_encodings(imgS, faceCurFrame)

            known_face_detected = False
            for encodeFace, faceLoc in zip(encodeCurFrame, faceCurFrame):
                matches = face_recognition.compare_faces(encodeListKnown, encodeFace)
                faceDis = face_recognition.face_distance(encodeListKnown, encodeFace)
                matchIndex = np.argmin(faceDis)

                if matches[matchIndex]:
                    known_face_detected = True
                    last_face_detected_time = time.time()
                    top, right, bottom, left = faceLoc
                    top *= 4
                    right *= 4
                    bottom *= 4
                    left *= 4
                    cv2.rectangle(img, (left, top), (right, bottom), (0, 255, 0), 2)

            # Hand gesture recognition if a known face is detected
            if known_face_detected:
                result = hands.process(framergb)
                className = ''

                if result.multi_hand_landmarks:
                    for handLms in result.multi_hand_landmarks:
                        landmarks = [[int(lm.x * img.shape[1]), int(lm.y * img.shape[0])] for lm in handLms.landmark]
                        mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

                        prediction = model.predict([landmarks])
                        classID = np.argmax(prediction)
                        className = classNames[classID]

                if className:
                    if len(gesture_buffer) < debounce_frame_count:
                        gesture_buffer.append(className)
                    else:
                        gesture_buffer.pop(0)
                        gesture_buffer.append(className)

                    most_common_gesture = max(set(gesture_buffer), key=gesture_buffer.count)
                    if gesture_buffer.count(most_common_gesture) > debounce_frame_count // 2:
                        confirmed_gesture = most_common_gesture

                if confirmed_gesture == 'stop':
                    tello.send_rc_control(0, 0, 0, 0)
                elif confirmed_gesture == 'thumbs up':
                    tello.flip_forward()
                elif confirmed_gesture == 'peace':
                    tello.land()
                    break

                if confirmed_gesture != 'stop':
                    tello.send_rc_control(0, 5, 0, 0)

                cv2.putText(img, confirmed_gesture, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            else:
                # If no known face is detected, check if 10 seconds have passed
                if time.time() - last_face_detected_time > 20:
                    tello.land()
                    break

            cv2.imshow('Tello Gesture Control', img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print('No frame from Tello')
            break

finally:
    tello.land()
    tello.end()
    cv2.destroyAllWindows()
