import cv2
import serial
import time
import numpy as np
import random

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/cu.usbmodem1201' 
BAUD_RATE = 9600
PROTOTXT_PATH = "deploy.prototxt.txt"
MODEL_PATH = "res10_300x300_ssd_iter_140000.caffemodel"
CONFIDENCE_THRESHOLD = 0.5
TILT_SERVO_MIN = 50
TILT_SERVO_MAX = 140
PAN_SERVO_MIN = 50
PAN_SERVO_MAX = 130

SACCADE_SPEED = 0.10 
# --- END CONFIGURATION ---

def send_command(arduino, pan_angle, tilt_angle):
    command = f"<{int(pan_angle)},{int(tilt_angle)}>\n"
    arduino.write(command.encode('utf-8'))

# --- Main script with state machine ---
arduino = None
cap = None
try:
    print("Attempting to connect to Arduino...")
    arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
    time.sleep(2)
    print("--> Serial connection established.")

    print("Attempting to load face detector model...")
    net = cv2.dnn.readNetFromCaffe(PROTOTXT_PATH, MODEL_PATH)
    print("--> Model files loaded successfully.")

    print("Attempting to open webcam...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened(): raise IOError("Cannot open webcam")
    print("--> Webcam opened successfully.")

    current_mode = 'IDLE'
    idle_sub_mode = 'FIXATING'
    current_pan = 90
    current_tilt = 90
    
    target_pan = 90
    target_tilt = 90
    last_saccade_time = time.time()
    saccade_interval = random.uniform(2.0, 5.0)

    while True:
        ret, frame = cap.read()
        if not ret: break

        frame = cv2.flip(frame, 1)
        (h, w) = frame.shape[:2]
        
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        net.setInput(blob)
        detections = net.forward()

        face_found = False
        for i in range(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > CONFIDENCE_THRESHOLD:
                face_found = True
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
                cv2.putText(frame, 'FACE DETECTED', (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                break

        if face_found:
            if current_mode != 'FACE_DETECTED':
                print("Face detected! Freezing movement.")
                current_mode = 'FACE_DETECTED'
        else:
            if current_mode != 'IDLE':
                print("Face lost. Resuming idle movement.")
                current_mode = 'IDLE'
                idle_sub_mode = 'FIXATING'
                last_saccade_time = time.time()

        if current_mode == 'IDLE':
            if idle_sub_mode == 'FIXATING':
                if time.time() - last_saccade_time > saccade_interval:
                    target_pan = random.randint(PAN_SERVO_MIN, PAN_SERVO_MAX)
                    target_tilt = random.randint(TILT_SERVO_MIN, TILT_SERVO_MAX)
                    idle_sub_mode = 'MOVING'
                    
            elif idle_sub_mode == 'MOVING':
                current_pan = (1 - SACCADE_SPEED) * current_pan + SACCADE_SPEED * target_pan
                current_tilt = (1 - SACCADE_SPEED) * current_tilt + SACCADE_SPEED * target_tilt

                if abs(current_pan - target_pan) < 2 and abs(current_tilt - target_tilt) < 2:
                    current_pan = target_pan
                    current_tilt = target_tilt
                    idle_sub_mode = 'FIXATING'
                    last_saccade_time = time.time()
                    saccade_interval = random.uniform(2.0, 5.0)

            if arduino:
                send_command(arduino, current_pan, current_tilt)
        
        cv2.imshow("Eyeball Bot", frame)
        
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"\n--- AN ERROR OCCURRED ---\nError: {e}\n-------------------------\n")

finally:
    print("Closing resources.")
    if cap is not None: cap.release()
    cv2.destroyAllWindows()
    if arduino is not None:
        send_command(arduino, 90, 90)
        arduino.close()
        print("Serial connection closed.")