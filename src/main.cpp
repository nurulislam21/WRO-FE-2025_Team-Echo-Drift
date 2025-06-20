#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

// Ultrasonic Sensor Pins
#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
    NewPing(9, 9, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
    NewPing(5, 5, MAX_DISTANCE),
    NewPing(11, 11, MAX_DISTANCE)};

// Servo Pin
#define SERVO_PIN 6

// Target distance from each wall (in cm)
#define TARGET_DISTANCE 200

Servo steeringServo;
// PID Variables
float errors = 0, PID_output = 0, integral = 0, derivative = 0, previous_errors = 0, present_position = 0;

// Servo center position (may need adjustment)
int servoCenter = 90;

int error_F();
void motor(int speedPercent);
void setup()
{
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate

  // Initialize servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(servoCenter);

  delay(1000); // Initial delay for stabilization
}

int commandToCode(const String &cmd)
{
  if (cmd == "right")
    return 1;
  if (cmd == "left")
    return 2;
  if (cmd == "centered")
    return 3;
  if (cmd == "stop")
    return 4;
  if (cmd == "park")
    return 5;
  return 0; // Default: PID control
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    switch (commandToCode(command))
    {
    case 1: // right
      steeringServo.write(130);
      motor(100);
      break;

    case 2: // left
      steeringServo.write(50);
      motor(100);
      break;

    case 3: // centered
      steeringServo.write(90);
      motor(100);
      break;

    case 4: // stop
      steeringServo.write(90);
      motor(0);
      while (true)
        ; // Halt
      break;

    case 5: // park
      // parking logic
      break;

    default: // PID auto steering
      PID_output = (3 * error_F()) + (30 * derivative);
      int servoPosition = constrain(servoCenter + PID_output, 0, 180);
      steeringServo.write(servoPosition);
      motor(100);
      // Serial.print(" | Servo: ");
      // Serial.println(servoPosition);
      break;
    }
  }
  else
  {
    PID_output = (3 * error_F()) + (30 * derivative);
    int servoPosition = constrain(servoCenter + PID_output, 0, 180);
    steeringServo.write(servoPosition);
    motor(100);
  }
}

float getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2; // Convert to cm

  // Filter out unrealistic values
  if (distance < 2 || distance > 200)
  {
    distance = TARGET_DISTANCE; // Return target if reading is invalid
  }

  return distance;
}
int error_F()
{
  // Read distances from both sensors
  float leftDistance = sonar[1].ping_cm();
  if (leftDistance == 0)
  {
    leftDistance = TARGET_DISTANCE; // Default to target distance if reading is invalid
  }
  float rightDistance = sonar[3].ping_cm();
  if (rightDistance == 0)
  {
    rightDistance = TARGET_DISTANCE; // Default to target distance if reading is invalid
  }
  // Calculate error (difference from target)
  errors = rightDistance - leftDistance;
  integral = integral + errors;
  derivative = errors - previous_errors;
  previous_errors = errors;
  return errors;
}

#define PWML 10
#define IN1L 9
#define IN2L 8

void motor(int speedPercent)
{
  int speedPWM = constrain((speedPercent * 255) / 100, -255, 255);

  pinMode(PWML, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  if (speedPWM > 0)
  {
    digitalWrite(IN1L, HIGH);
    digitalWrite(IN2L, LOW);
    analogWrite(PWML, speedPWM);
  }
  else if (speedPWM < 0)
  {
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, HIGH);
    analogWrite(PWML, -speedPWM);
  }
  else
  {
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, LOW);
    analogWrite(PWML, 255); // Brake mode, or stop
  }
}
