#include <Arduino.h>

#include <Servo.h>

// Ultrasonic Sensor Pins
#define TRIG_PIN_LEFT 2
#define ECHO_PIN_LEFT 3
#define TRIG_PIN_RIGHT 4
#define ECHO_PIN_RIGHT 5

// Servo Pin
#define SERVO_PIN 9

// Target distance from each wall (in cm)
#define TARGET_DISTANCE 200

Servo steeringServo;

// PID Variables
float errors = 0, PID_output = 0, integral = 0, derivative = 0, previous_errors = 0, present_position = 0;

// Servo center position (may need adjustment)
int servoCenter = 90;

int error_F();

void setup() {
  Serial.begin(9600);
  
  // Initialize ultrasonic sensors
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  
  // Initialize servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(servoCenter);
  
  delay(1000); // Initial delay for stabilization
}

void loop() {
  // Calculate servo position (center ± output)
  PID_output = (3 * error_F()) + (30 * derivative);
  int servoPosition = servoCenter + PID_output;
  // Constrain servo position to safe limits (typically 0-180)
  servoPosition = constrain(servoPosition, 0, 180);
  
  // Move servo
  steeringServo.write(servoPosition);
  
  
  Serial.print(" | Servo: ");
  Serial.println(servoPosition);
  
  // Small delay to prevent flooding the serial monitor
  delay(50);
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2; // Convert to cm
  
  // Filter out unrealistic values
  if (distance < 2 || distance > 200) {
    distance = TARGET_DISTANCE; // Return target if reading is invalid
  }
  
  return distance;
}
int error_F()
{
  // Read distances from both sensors
  float leftDistance = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  float rightDistance = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  
  // Calculate error (difference from target)
  errors = rightDistance - leftDistance;
  integral = integral + errors;
  derivative = errors - previous_errors;
  previous_errors = errors;
  return errors;
}