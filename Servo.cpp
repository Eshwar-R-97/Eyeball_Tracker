#include <Servo.h>

// Create two servo objects
Servo tiltServo;  // Vertical movement
Servo panServo;   // Horizontal movement

// Define the pins the servos are connected to
const int tiltPin = 9;  // Connect vertical servo signal wire here
const int panPin = 10;  // Connect horizontal servo signal wire here

void setup() {
  // Start serial communication at 9600 baud rate (must match Python script)
  Serial.begin(9600);
  
  // Attach the servos to their pins
  tiltServo.attach(tiltPin);
  panServo.attach(panPin);
  
  // Move servos to a neutral starting position
  tiltServo.write(90);
  panServo.write(90);

  Serial.println("Arduino is ready to receive commands.");
}

void loop() {
  // Check if there is data available to read from the serial port
  if (Serial.available() > 0) {
    // Read the incoming string until a newline character is received
    String command = Serial.readStringUntil('\n');

    // Find the comma that separates the two angle values
    int commaIndex = command.indexOf(',');

    // Check if the command is in the expected format: <pan,tilt>
    if (command.startsWith("<") && command.endsWith(">") && commaIndex != -1) {
      
      // Extract the pan angle string (from after '<' up to the comma)
      String panString = command.substring(1, commaIndex);
      
      // Extract the tilt angle string (from after the comma up to '>')
      String tiltString = command.substring(commaIndex + 1, command.length() - 1);

      // Convert the strings to integers
      int panAngle = panString.toInt();
      int tiltAngle = tiltString.toInt();

      // Constrain the angles to your safe operating range as a final safety measure
      panAngle = constrain(panAngle, 50, 130);   // Horizontal range
      tiltAngle = constrain(tiltAngle, 50, 140); // Vertical range

      // Write the final angles to the servos
      panServo.write(panAngle);
      tiltServo.write(tiltAngle);
    }
  }
}