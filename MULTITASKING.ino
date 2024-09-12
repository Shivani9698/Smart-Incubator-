#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// Pin assignments
#define DIR 27
#define STEP 26
#define DOOR_SENSOR_PIN 13  // Pin for door sensor (changed from conflicting pin 26)
#define DOOR_RELAY_PIN 32   // Pin for door relay (adjusted)
#define ONE_WIRE_BUS 4      // Pin for DS18B20 sensors
#define RELAY_PIN 25        // Pin to control the heating relay

const int steps_per_rev = 50000;
int a = 1000;

// Task Handles
TaskHandle_t StepperMotorTaskHandle = NULL;
TaskHandle_t DoorSensorTaskHandle = NULL;
TaskHandle_t TempControlTaskHandle = NULL;

// PID variables
double Setpoint = 37.5;  // Desired temperature for incubation
double Input, Output;    // PID Input and Output
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // PID Controller

// OneWire instance and temperature sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
double temp[4];

// Function Prototypes
void StepperMotorTask(void *pvParameters);
void DoorSensorTask(void *pvParameters);
void TempControlTask(void *pvParameters);
void printAddress(DeviceAddress deviceAddress);

void setup() {
  Serial.begin(115200);

  // Initialize stepper motor pins
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);

  // Initialize door sensor and relay pins
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(DOOR_RELAY_PIN, OUTPUT);
  digitalWrite(DOOR_RELAY_PIN, LOW);

  // Initialize temperature sensors and relay
  sensors.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Print sensor addresses for debugging
  DeviceAddress sensorAddress;
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    if (sensors.getAddress(sensorAddress, i)) {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" Address: ");
      printAddress(sensorAddress);
    }
  }

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  // Create FreeRTOS tasks
  xTaskCreate(StepperMotorTask, "StepperMotorTask", 10000, NULL, 1, &StepperMotorTaskHandle);
  xTaskCreate(DoorSensorTask, "DoorSensorTask", 10000, NULL, 1, &DoorSensorTaskHandle);
  xTaskCreate(TempControlTask, "TempControlTask", 10000, NULL, 1, &TempControlTaskHandle);
}

void loop() {
  // Nothing in the main loop since FreeRTOS is managing the tasks
}

void StepperMotorTask(void *pvParameters) {
  while (true) {
    // Clockwise rotation
    digitalWrite(DIR, LOW);
    Serial.println("Spinning Clockwise...");
    for (int i = 0; i < steps_per_rev; i++) {
      digitalWrite(STEP, HIGH);
      delayMicroseconds(a);
      digitalWrite(STEP, LOW);
      delayMicroseconds(a);
     // Serial.println(i);
    }
    delay(1000);

    // Counterclockwise rotation
    digitalWrite(DIR, HIGH);
    Serial.println("Spinning Anti-Clockwise...");
    for (int i = 0; i < steps_per_rev; i++) {
      digitalWrite(STEP, HIGH);
      delayMicroseconds(a);
      digitalWrite(STEP, LOW);
      delayMicroseconds(a);
    }
    delay(1000);
  }
}

void DoorSensorTask(void *pvParameters) {
  while (true) {
    int doorState = digitalRead(DOOR_SENSOR_PIN);
    if (doorState == LOW) {
      digitalWrite(DOOR_RELAY_PIN, HIGH);
      Serial.println("Door is closed. Door Relay OFF.");
    } else {
      digitalWrite(DOOR_RELAY_PIN, LOW);
      Serial.println("Door is open. Door Relay ON.");
    }
    vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 2 seconds
  }
}

void TempControlTask(void *pvParameters) {
  while (true) {
    sensors.requestTemperatures();

    // Read temperatures and calculate average
    Input = 0;
    int validSensorCount = 0;
    for (int i = 0; i < sensors.getDeviceCount(); i++) {
      temp[i] = sensors.getTempCByIndex(i);
      if (temp[i] != -127.00 && temp[i] != 85.00) {  // Only valid temperatures
        Input += temp[i];
        validSensorCount++;
      }
    }

    if (validSensorCount > 0) {
      Input /= validSensorCount;
      Serial.print("Average Temperature: ");
      Serial.println(Input);

      // Run PID control
      myPID.Compute();

      if (Output > 100) {
        digitalWrite(RELAY_PIN, LOW);
        Serial.println("Relay ON: Heating...");
      } else {
        digitalWrite(RELAY_PIN, HIGH);
        Serial.println("Relay OFF: Not Heating...");
      }
    } else {
      Serial.println("No valid sensor readings. Skipping this cycle.");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
  }
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}
