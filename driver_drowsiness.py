import cv2
import numpy as np
import dlib
from imutils import face_utils
import pygame
import os

# Initialize pygame mixer
pygame.mixer.init()




# Get the absolute path of the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Load the sound files using relative paths
alarmsound = pygame.mixer.Sound(os.path.join(script_dir, "Sounds", "leftright.wav"))
sleepingsound = pygame.mixer.Sound(os.path.join(script_dir, "Sounds", "sleeping.wav"))
jawnsound = pygame.mixer.Sound(os.path.join(script_dir, "Sounds", "Beep.wav"))

# Initializing the camera and taking the instance
cap = cv2.VideoCapture(0)

# Initializing the face detector and landmark detector
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(os.path.join(script_dir, "Models", "shape_predictor_68_face_landmarks.dat"))

# Status marking for current state
sleep = 0
drowsy = 0
active = 0
status = ""
color = (0, 0, 0)

# Counters for blinks and jaw movements
blink_count = 0
jaw_movement_count = 0

# Flags to track the state
previous_left_blink = False
previous_right_blink = False
previous_jaw_movement = False

def compute(ptA, ptB):
    dist = np.linalg.norm(ptA - ptB)
    return dist

def blinked(a, b, c, d, e, f):
    up = compute(b, d) + compute(c, e)
    down = compute(a, f)
    ratio = up / (2.0 * down)

    # Checking if it is blinked
    if ratio > 0.18:
        return 2
    elif 0.10 < ratio <= 0.18:  # Adjusted threshold for better detection
        return 1
    else:
        return 0

def jawing(a, b, threshold=10):  # Adjusted threshold for better detection
    # Compute the vertical movement of the jaw
    dist = compute(a, b)
    #print("Distance:", dist)  # Print distance for debugging
    #print("Threshold:", threshold)  # Print threshold for debugging
    return dist > 30

def determine_looking_direction(landmarks):
    # Get the coordinates of specific landmarks
    left_eye_center = np.mean([landmarks[36], landmarks[37], landmarks[38], landmarks[39]], axis=0)
    right_eye_center = np.mean([landmarks[42], landmarks[43], landmarks[44], landmarks[45]], axis=0)
    nose_bridge = landmarks[30]

    # Calculate the direction vector from the nose bridge to the midpoint between the eyes
    direction_vector = ((left_eye_center + right_eye_center) / 2) - nose_bridge

    # Calculate the angle of the direction vector
    angle = np.arctan2(direction_vector[1], direction_vector[0]) * 180 / np.pi
    #print(angle)

    # Determine the direction based on the angle
    #-95 center, -110 left,-80 right
    if -80 > angle > -110:
        return "Center"
    elif -70 >= angle >=-80:
        return "Right"
    elif angle >-70:
        return "Right Alarm"
    elif angle<=-120:
        return "Left Alarm"
    else:
        return "Left"





while True:
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)
    face_frame = frame.copy()  # Initialize face_frame with the current frame

    # Detected face in faces array
    for face in faces:
        x1 = face.left()
        y1 = face.top()
        x2 = face.right()
        y2 = face.bottom()

        cv2.rectangle(face_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        landmarks = predictor(gray, face)
        landmarks = face_utils.shape_to_np(landmarks)

        # The numbers are actually the landmarks which will show eye
        left_blink = blinked(landmarks[36], landmarks[37],
                             landmarks[38], landmarks[41], landmarks[40], landmarks[39])
        right_blink = blinked(landmarks[42], landmarks[43],
                              landmarks[44], landmarks[47], landmarks[46], landmarks[45])
        
        # Checking for jaw movement
        jaw_moved = jawing(landmarks[67], landmarks[63])

        # Now judge what to do for the eye blinks
        if left_blink == 0 or right_blink == 0:
            sleep += 1
            drowsy = 0
            active = 0
            if sleep > 6:
                status = "SLEEPING !!!"
                color = (255, 0, 0)

        elif left_blink == 1 or right_blink == 1:
            sleep = 0
            active = 0
            drowsy += 1
            if drowsy > 6:
                status = "Drowsy !"
                color = (0, 0, 255)

        else:
            drowsy = 0
            sleep = 0
            active += 1
            if active > 6:
                status = ""
                color = (0, 255, 0)

                
        
        # Increment blink counter if a blink is detected
        if (left_blink == 2 and not previous_left_blink) or (right_blink == 2 and not previous_right_blink):
            blink_count += 1

        # Update the blink state
        previous_left_blink = left_blink == 2
        previous_right_blink = right_blink == 2

        # Increment jaw movement counter if jaw movement is detected
        if jaw_moved and not previous_jaw_movement:
            jaw_movement_count += 1
            jawnsound.play()

        # Update the jaw movement state
        previous_jaw_movement = jaw_moved

        # Inside the main loop
        direction = determine_looking_direction(landmarks)
        cv2.putText(frame, f"Looking Direction: {direction}", (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        if direction == "Left Alarm":
            alarmsound.play()
        elif direction == "Right Alarm":
            alarmsound.play()

        if status=="SLEEPING !!!":
                sleepingsound.play()   


        cv2.putText(frame, status, (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        cv2.putText(frame, f"Blinks: {blink_count}", (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(frame, f"Jaw Movements: {jaw_movement_count}", (10, 400), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)

        for n in range(0, 68):
            (x, y) = landmarks[n]
            cv2.circle(face_frame, (x, y), 1, (255, 255, 255), -1)

    cv2.imshow("Frame", frame)
    if len(faces) > 0:  # Display face_frame only if a face is detected
        cv2.imshow("Result of detector", face_frame)
    key = cv2.waitKey(1)
    if key == 27:
        break


cap.release()
cv2.destroyAllWindows()
