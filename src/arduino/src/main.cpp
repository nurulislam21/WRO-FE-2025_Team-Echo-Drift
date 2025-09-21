#include <Arduino.h>
#include <Servo.h>

#define BUTTON_PIN 4 // D4
#define SHARP_IR A4
#define TRIGGER_PIN A2
#define ECHO_PIN A3
#define PWML 11
#define IN1L 6
#define IN2L 7

Servo steeringServo;

int encoderPin1 = 2;            // Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3;            // Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncoded = 0;   // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value

unsigned long lastReceivedTime = 0; // Time of last valid serial input
const unsigned long timeout = 1000; // 1 second timeout
int currentAngle = 95;              // Default to 90

int servoMaxLeft = 20;
int servoMaxRight = 170;

void motor(int speedPercent);
void sw();
void updateEncoder();
void setup()
{
    Serial.begin(115200);
    steeringServo.attach(5);
    steeringServo.write(currentAngle); // Initialize to 90
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Enable internal pull-up resistor
    pinMode(encoderPin1, INPUT_PULLUP);
    pinMode(encoderPin2, INPUT_PULLUP);

    digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); // turn pullup resistor on

    // call updateEncoder() when any high/low changed seen
    // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
    attachInterrupt(0, updateEncoder, CHANGE);
    attachInterrupt(1, updateEncoder, CHANGE);
    // Wait for serial data
    while (!Serial.available())
    {
        delay(10);
    }

    // Once serial is available, perform the servo movement
    steeringServo.write(120);
    delay(500);
    steeringServo.write(60);
    delay(500);
    steeringServo.write(95);
    delay(50);
    sw();
}
void loop()
{
    String inputData = "";
    // Read serial input if available
    while (Serial.available())
    {
        inputData = Serial.readStringUntil('\n');

        // parse space separated values, e.g., "speed,steps,angle"
        char *payload = strdup(inputData.c_str()); // Copy data to mutable string
        String speed = strtok(payload, ",");
        String stepsStr = strtok(NULL, ",");
        String angleStr = strtok(NULL, ",");

        int speedValue = atoi(speed.c_str());
        int stepsValue = atoi(stepsStr.c_str());
        int angle = atoi(angleStr.c_str());

        currentAngle = constrain(angle, servoMaxLeft, servoMaxRight);

        if (stepsValue > 0)
        {
            steeringServo.write(currentAngle);
            delay(100);
            moveEncoder(speedValue, stepsValue);
            delay(100);
            Serial.println("DONE");
        }

        steeringServo.write(currentAngle);
        lastReceivedTime = millis(); // Reset timeout timer

        motor(speedValue);
        free(payload); // Free allocated memory

        if (digitalRead(BUTTON_PIN) == LOW)
        {
            delay(200);
            sw(); // Wait for button press to restart
        }
    }

    // Timeout logic: no data received recently
    if (millis() - lastReceivedTime > timeout)
    {
        if (currentAngle != 95)
        {
            currentAngle = 95;
            steeringServo.write(currentAngle);
            Serial.println("No serial input — resetting to 95°");
        }
        motor(-50);
        delay(100);
        motor(0);
        delay(100);
        while (true)
        {
        }
    }
}

void sw()
{
    // Wait for the button to be pressed (goes LOW)
    while (digitalRead(BUTTON_PIN) == HIGH)
    {
        delay(10); // Small delay to reduce CPU load
    }

    // Debounce delay to register the button press
    delay(50);

    // Wait for the button to be released (goes HIGH)
    while (digitalRead(BUTTON_PIN) == LOW)
    {
        delay(10); // Small delay to reduce CPU load
    }

    Serial.println("START");
    // Final debounce delay to ensure clean button release
    delay(50);
}

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

void updateEncoder()
{
    int MSB = digitalRead(encoderPin1); // MSB = most significant bit
    int LSB = digitalRead(encoderPin2); // LSB = least significant bit

    int encoded = (MSB << 1) | LSB;         // converting the 2 pin value to single number
    int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoderValue--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoderValue++;

    lastEncoded = encoded; // store this value for next time
}
volatile long encoderCount = 0;

void encoderISR()
{
    encoderCount++;
}
void resetEncoder()
{
    noInterrupts();
    encoderValue = 0;
    interrupts();
}

long getEncoder()
{
    long val;
    noInterrupts();
    val = encoderValue;
    interrupts();
    return val;
}

void moveEncoder(int speed, long targetPulses)
{
    resetEncoder();
    motor(speed);
    if (speed > 0)
    {
        while (getEncoder() < targetPulses)
        {
            // forward until reached
        }
    }
    else
    {
        while (getEncoder() > -targetPulses)
        {
            // backward until reached
        }
    }
    motor(0);
}