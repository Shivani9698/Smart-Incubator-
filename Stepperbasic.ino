#include <Arduino.h>

// Pin assignments
#define DIR 27
#define STEP 26

const int steps_per_rev = 500000;  // Number of steps per revolution
int a = 50000;  // Microseconds delay between steps

void setup() {
    Serial.begin(115200);

    // Initialize stepper motor pins
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);
}

void loop() {
    // Clockwise rotation
    digitalWrite(DIR, LOW);
    Serial.println("Spinning Clockwise...");
    for (int i = 0; i < steps_per_rev; i++) {
        digitalWrite(STEP, HIGH);
        delayMicroseconds(a);
        digitalWrite(STEP, LOW);
        delayMicroseconds(a);
    }

    delay(1000);  // Short delay before changing direction

    // Counterclockwise rotation
    digitalWrite(DIR, HIGH);
    Serial.println("Spinning Anti-Clockwise...");
    for (int i = 0; i < steps_per_rev; i++) {
        digitalWrite(STEP, HIGH);
        delayMicroseconds(a);
        digitalWrite(STEP, LOW);
        delayMicroseconds(a);
    }

    delay(1000);  // Delay before the next loop
}
