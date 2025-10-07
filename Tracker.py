import cv2
import serial
import time
import numpy as np

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/cu.usbmodem1201' # this is my port name on Mac, change as needed
BAUD_RATE = 9600
PROTOTXT_PATH = "deploy.prototxt.txt"
MODEL_PATH = "res10_300x300_ssd_iter_140000.caffemodel"
CONFIDENCE_THRESHOLD = 0.5
TILT_SERVO_MIN = 40
TILT_SERVO_MAX = 145
PAN_SERVO_MIN = 30
PAN_SERVO_MAX = 150
SMOOTHING_FACTOR = 0.1
SEND_INTERVAL = 0.05 # seconds between sending commands to Arduino
# --- END CONFIGURATION ---

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def send_command(arduino, pan_angle, tilt_angle):
    command = f"<{int(pan_angle)},{int(tilt_angle)}>\n"
    arduino.write(command.encode('utf-8'))
    print(f"Python Sent: {command.strip()}")

# --- Main script with Arduino listener ---
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

    current_pan_angle = 90
    current_tilt_angle = 90
    last_send_time = 0

    while True:
        ret, frame = cap.read()
        if not ret: break

        frame = cv2.flip(frame, 1)
        (h, w) = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        net.setInput(blob)
        detections = net.forward()

        best_confidence = 0
        best_box = None
        
        for i in range(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > CONFIDENCE_THRESHOLD and confidence > best_confidence:
                best_confidence = confidence
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                best_box = box.astype("int")

        if best_box is not None:
            (startX, startY, endX, endY) = best_box
            center_x = (startX + endX) // 2
            center_y = (startY + endY) // 2
            
            # --- CORRECTED MAPPING LOGIC ---
            target_pan_angle = map_value(center_x, 0, w, PAN_SERVO_MIN, PAN_SERVO_MAX)
            target_tilt_angle = map_value(center_y, 0, h, TILT_SERVO_MAX, TILT_SERVO_MIN)
            
            current_pan_angle = (1 - SMOOTHING_FACTOR) * current_pan_angle + SMOOTHING_FACTOR * target_pan_angle
            current_tilt_angle = (1 - SMOOTHING_FACTOR) * current_tilt_angle + SMOOTHING_FACTOR * target_tilt_angle
            
            current_time = time.time()
            if arduino and (current_time - last_send_time) > SEND_INTERVAL:
                send_command(arduino, current_pan_angle, current_tilt_angle)
                last_send_time = time.time()

            text = "{:.2f}%".format(best_confidence * 100)
            y = startY - 10 if startY - 10 > 10 else startY + 10
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
            cv2.putText(frame, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

        cv2.imshow("DNN Head Tracker", frame)
        
        if arduino and arduino.in_waiting > 0:
            response = arduino.readline().decode('utf-8').strip()
            if response:
                print(f"Arduino says: {response}")
        
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