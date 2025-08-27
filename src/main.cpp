#include <Arduino.h>
#include <Servo.h>

Servo steeringServo;

unsigned long lastReceivedTime = 0;  // Time of last valid serial input
const unsigned long timeout = 1000;  // 1 second timeout
int currentAngle = 95;               // Default to 90
#define BUTTON_PIN 2  // D2
void motor(int speedPercent);
void sw();
void setup() {
  Serial.begin(115200);
  steeringServo.attach(5);
  steeringServo.write(currentAngle);  // Initialize to 90
  pinMode(6, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Enable internal pull-up resistor
  digitalWrite(6, HIGH);
  sw();
}
void loop() {
  static char input[6];
  static byte pos = 0;

  // Read serial input if available
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      input[pos] = '\0';
      int angle = atoi(input);
      currentAngle = angle;  // Update current angle
      steeringServo.write(currentAngle);
      Serial.print("Got angle: ");
      Serial.println(currentAngle);
      pos = 0;
      lastReceivedTime = millis();  // Reset timeout timer
    } else if (pos < sizeof(input) - 1) {
      input[pos++] = c;
    }
    motor(100);
  }

  // Timeout logic: no data received recently
  if (millis() - lastReceivedTime > timeout) {
    if (currentAngle != 95) {
      currentAngle = 95;
      steeringServo.write(currentAngle);
      Serial.println("No serial input — resetting to 95°");
    }
    motor(0);
  }
}

void sw() {
  // Wait for the button to be pressed (goes LOW)
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(10); // Small delay to reduce CPU load
  }

  // Debounce delay to register the button press
  delay(50);

  // Wait for the button to be released (goes HIGH)
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(10); // Small delay to reduce CPU load
  }

  // Final debounce delay to ensure clean button release
  delay(50);
}
#define PWML 9
#define IN1L 7
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