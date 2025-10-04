import cv2
import serial
import time

# --- CONFIGURATION ---
# !! IMPORTANT !!
# !! Replace 'YOUR_PORT_NAME' with the port your Arduino is on.
SERIAL_PORT = 'YOUR_PORT_NAME'
BAUD_RATE = 9600

# Servo angle ranges
TILT_SERVO_MIN = 50
TILT_SERVO_MAX = 140
PAN_SERVO_MIN = 50
PAN_SERVO_MAX = 130
# --- END CONFIGURATION ---


def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def send_command(arduino, pan_angle, tilt_angle):
    command = f"<{pan_angle},{tilt_angle}>\\n"
    arduino.write(command.encode('utf-8'))
    print(f"Sent: {command.strip()}")

# --- SERIAL COMMUNICATION SETUP ---
try:
    arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
    time.sleep(2)
    print("Serial connection established.")
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    # We can continue without an Arduino for visual testing
    arduino = None

# --- OPENCV SETUP ---
face_cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(face_cascade_path)
cap = cv2.VideoCapture(0)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


# --- MAIN TRACKING LOOP ---
try:
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))

        if len(faces) > 0:
            x, y, w, h = faces[0]
            center_x = x + w // 2
            center_y = y + h // 2

            # --- VISUALS ARE DRAWN HERE ---
            # 1. This line draws the green rectangle around the face
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 2. NEW: This line adds the "Head" text label above the box
            cv2.putText(frame, 'Head', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            # --- END VISUALS ---

            if arduino:
                pan_angle = map_value(center_x, 0, frame_width, PAN_SERVO_MAX, PAN_SERVO_MIN)
                tilt_angle = map_value(center_y, 0, frame_height, TILT_SERVO_MIN, TILT_SERVO_MAX)
                send_command(arduino, pan_angle, tilt_angle)
        
        # This line displays the window with the webcam feed and all the drawings
        cv2.imshow('Head Tracker', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # --- CLEANUP ---
    print("Closing resources.")
    cap.release()
    cv2.destroyAllWindows()
    if arduino:
        send_command(arduino, 90, 90) # Center servos on exit
        arduino.close()
        print("Serial connection closed.")
        print(f"Webcam resolution: {frame_width}x{frame_height}")