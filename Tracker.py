import cv2
import serial
import time

# --- CONFIGURATION ---
# !! IMPORTANT !!
# !! Replace 'YOUR_PORT_NAME' with the port your Arduino is on.
# !! Find this in the Arduino IDE under Tools > Port.
SERIAL_PORT = 'YOUR_PORT_NAME'
BAUD_RATE = 9600

# Servo angle ranges (provided by you)
TILT_SERVO_MIN = 50   # Vertical servo (pos1) min angle
TILT_SERVO_MAX = 140  # Vertical servo (pos1) max angle
PAN_SERVO_MIN = 50    # Horizontal servo (pos2) min angle
PAN_SERVO_MAX = 130   # Horizontal servo (pos2) max angle
# --- END CONFIGURATION ---


def map_value(value, in_min, in_max, out_min, out_max):
    """Helper function to map a value from one range to another."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def send_command(arduino, pan_angle, tilt_angle):
    """Sends the calculated pan and tilt angles to the Arduino."""
    command = f"<{pan_angle},{tilt_angle}>\\n"
    arduino.write(command.encode('utf-8'))
    print(f"Sent: {command.strip()}")

# 1. --- SERIAL COMMUNICATION SETUP ---
try:
    arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
    # Allow time for the connection to establish
    time.sleep(2)
    print("Serial connection established.")
except Exception as e:
    print(f"Error: Could not connect to {SERIAL_PORT}. Please check the port name and permissions.")
    print(e)
    exit()

# 2. --- OPENCV SETUP ---
# Load the pre-trained Haar Cascade for face detection
# This file comes with the OpenCV library
face_cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(face_cascade_path)

# Start video capture from the default webcam (usually camera 0)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    arduino.close()
    exit()

# Get frame dimensions
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Webcam resolution: {frame_width}x{frame_height}")


# 3. --- MAIN TRACKING LOOP ---
try:
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Flip the frame horizontally for a more intuitive "mirror" effect
        frame = cv2.flip(frame, 1)

        # Convert the frame to grayscale for the face detector
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces in the grayscale frame
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))

        if len(faces) > 0:
            # For simplicity, track the first face found
            x, y, w, h = faces[0]

            # Calculate the center of the face
            center_x = x + w // 2
            center_y = y + h // 2

            # Map the face center coordinates to your custom servo angle ranges
            # NOTE: We swap the pan range to make the movement intuitive.
            # When face is left (low X), servo angle is high (turns left).
            pan_angle = map_value(center_x, 0, frame_width, PAN_SERVO_MAX, PAN_SERVO_MIN)
            tilt_angle = map_value(center_y, 0, frame_height, TILT_SERVO_MIN, TILT_SERVO_MAX)
            
            # Send the calculated angles to the Arduino
            send_command(arduino, pan_angle, tilt_angle)

            # Draw a rectangle around the detected face for visual feedback
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Display the resulting frame in a window
        cv2.imshow('Head Tracker', frame)

        # Check if the 'q' key was pressed to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting...")
            break

finally:
    # 4. --- CLEANUP ---
    print("Closing resources.")
    cap.release()
    cv2.destroyAllWindows()
    # Send a final command to center the servos before closing
    send_command(arduino, 90, 90)
    arduino.close()
    print("Serial connection closed.")