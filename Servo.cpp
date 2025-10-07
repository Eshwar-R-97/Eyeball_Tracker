#include <Servo.h>

Servo tiltServo;
Servo panServo;

const int tiltPin = 9;
const int panPin = 10;

char command_buffer[32];

void setup() {
  Serial.begin(9600);
  tiltServo.attach(tiltPin);
  panServo.attach(panPin);
  tiltServo.write(90);
  panServo.write(90);
  Serial.println("Arduino is ready. Test sketch v2 loaded.");
}

void loop() {
  read_serial_command();
}

void read_serial_command() {
  static byte buffer_index = 0;

  while (Serial.available() > 0) {
    char incoming_char = Serial.read();

    if (incoming_char == '\n') {
      command_buffer[buffer_index] = '\0';
      parse_and_actuate(command_buffer);
      buffer_index = 0;
    }
    else {
      if (buffer_index < (sizeof(command_buffer) - 1)) {
        command_buffer[buffer_index++] = incoming_char;
      }
    }
  }
}

void parse_and_actuate(char* command) {
  int panAngle, tiltAngle;

  if (sscanf(command, "<%d,%d>", &panAngle, &tiltAngle) == 2) {
    
    panAngle = constrain(panAngle, 40, 140);
    tiltAngle = constrain(tiltAngle, 40, 145);

    // --- NEW ORDER OF OPERATIONS ---
    // 1. Send the confirmation message FIRST (low-power operation)
    Serial.print("Parsed -> P:");
    Serial.print(panAngle);
    Serial.print(", T:");
    Serial.println(tiltAngle);
    
    // 2. THEN, try to move the servos (high-power operation)
    panServo.write(panAngle);
    tiltServo.write(tiltAngle);
  }
}