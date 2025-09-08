#include <Arduino.h>
#include <Servo.h>

#define PWML 11
#define IN1L 6
#define IN2L 7
#define BUTTON_PIN 4 // D4

Servo steeringServo;

int encoderPin1 = 2;            // Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3;            // Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncoded = 0;   // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value

unsigned long lastReceivedTime = 0; // Time of last valid serial input
const unsigned long timeout = 1000; // 1 second timeout
int currentAngle = 95;              // Default to 90

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
    static byte pos = 0;

    // Read serial input if available
    while (Serial.available())
    {
        inputData = Serial.readStringUntil('\n');

        // parse space separated values, e.g., "speed,angle"
        char *payload = strdup(inputData.c_str()); // Copy data to mutable string
        String speed = strtok(payload, ",");
        String angleStr = strtok(NULL, ",");

        int speedValue = atoi(speed.c_str());
        int angle = atoi(angleStr.c_str());
        if (angle > 140)
        {
            angle = 140;
        }
        if (angle < 50)
        {
            angle = 50;
        }

        currentAngle = angle; // Update current angle
        steeringServo.write(currentAngle);
        Serial.print("Got angle: ");
        Serial.println(currentAngle);
        pos = 0;
        lastReceivedTime = millis(); // Reset timeout timer

        motor(speedValue);
        free(payload); // Free allocated memory
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
        while (1)
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