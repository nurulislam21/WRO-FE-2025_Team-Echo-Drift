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
void motor(int speedPercent);
void setup()
{
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
  float leftDistance = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  float rightDistance = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

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

void motor(int speedPercent) {
  int speedPWM = constrain((speedPercent * 255) / 100, -255, 255);

  pinMode(PWML, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  if (speedPWM > 0) {
    digitalWrite(IN1L, HIGH);
    digitalWrite(IN2L, LOW);
    analogWrite(PWML, speedPWM);
  } else if (speedPWM < 0) {
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, HIGH);
    analogWrite(PWML, -speedPWM);
  } else {
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, LOW);
    analogWrite(PWML, 255);  // Brake mode (if supported), or stop
  }
}

