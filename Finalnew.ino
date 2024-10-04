#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <esp_sleep.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <Arduino.h>
#include <Nextion.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Pin assignments for temperature, relay, door, stepper motor, LED
#define ONE_WIRE_BUS 4
#define RELAY_PIN 25
#define doorSensorPin 13
#define doorRelayPin 27
#define DIR 2
#define STEP 5
#define LED_PIN 12
#define NUM_PIXELS 55
#define DHT_PIN 18

// DHT sensor setup
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// PID variables
double Setpoint = 37.5;
double Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// DS18B20 OneWire and DallasTemperature setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// WiFi and HTTP setup
const char* serverName = "http://app.antzsystems.com/api/v1/iot/enclosure/metric/update";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800);  // IST Time Offset

// Stepper motor settings
const int steps_per_rev = 500000;
int stepDelay = 50000; // Microseconds between steps
double stepperRMS = 0; // RMS value for the stepper motor

// Function prototypes for tasks
void TemperatureTask(void *pvParameters);
void ServerTask(void *pvParameters);
void DoorTask(void *pvParameters);
void StepperTask(void *pvParameters);
void HumidityTask(void *pvParameters);
void updateLEDColor(double temperature);

// Nextion display
NexText t0 = NexText(1, 1, "t0"); // page 1, id 1 (temperature)
NexText t1 = NexText(1, 2, "t1"); // page 1, id 2 (humidity)

// NeoPixel strip
Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

TaskHandle_t serverTaskHandle; // Task handle for ServerTask
SemaphoreHandle_t nextionMutex; // Mutex to ensure Nextion communication safety

void setup() {
  Serial.begin(9600);

  // Initialize NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Initialize temperature sensor
  sensors.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetTunings(Kp, Ki, Kd);

  // Initialize DHT sensor
  dht.begin();

  // Initialize WiFi
  WiFiManager wifiManager;
  wifiManager.autoConnect("Incubator"); // Start the WiFi manager

  // Initialize NTP Client
  timeClient.begin();

  // Initialize door sensor and relay
  pinMode(doorSensorPin, INPUT_PULLUP);
  pinMode(doorRelayPin, OUTPUT);
  digitalWrite(doorRelayPin, LOW);

  // Initialize stepper motor pins
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);

  // Initialize Nextion display
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // Serial2 for Nextion (TX pin 17, RX pin 16)
  nexInit(); // Initialize Nextion display
  Serial.println("Nextion display initialized");

  // Initialize mutex for Nextion communication
  nextionMutex = xSemaphoreCreateMutex();

  // Create FreeRTOS tasks with sufficient stack size
  xTaskCreate(TemperatureTask, "Temperature Control", 15000, NULL, 4, NULL);
  xTaskCreate(ServerTask, "Server Task", 15000, NULL, 1, &serverTaskHandle);
  xTaskCreate(DoorTask, "Door Control", 10000, NULL, 0, NULL);
  xTaskCreate(StepperTask, "Stepper Control", 12000, NULL, 2, NULL);
  xTaskCreate(HumidityTask, "Humidity Task", 12000, NULL, 3, NULL);

  // Monitor heap space initially
  Serial.printf("Initial free heap: %d bytes\n", esp_get_free_heap_size());
}

void loop() {
  // Monitor heap space every 5 seconds
  Serial.printf("Free heap: %d bytes\n", esp_get_free_heap_size());
  vTaskDelay(pdMS_TO_TICKS(5000)); // Delay in main loop
}

// Task 1: Temperature Control with Random Values (for now)
void TemperatureTask(void *pvParameters) {
  while (1) {
    int randomTemp = random(20, 40);  // Generate a random temperature value
    Serial.printf("Random Temperature: %d\n", randomTemp);
    
    // Simulate the PID control using random temperature
    Input = randomTemp;
    myPID.Compute();

    if (Output > 100) {
      digitalWrite(RELAY_PIN, HIGH); // Heating
      Serial.println("Heating ON");
    } else {
      digitalWrite(RELAY_PIN, LOW);  // Heating OFF
      Serial.println("Heating OFF");
    }

    updateLEDColor(Input); // Update LED color based on temperature

    // Send temperature to Nextion safely
    if (xSemaphoreTake(nextionMutex, pdMS_TO_TICKS(100))) {
      bool success = t0.setText(String(randomTemp).c_str());
      if (success) {
        Serial.println("Temperature updated on display");
      } else {
        Serial.println("Failed to update temperature on display");
      }
      xSemaphoreGive(nextionMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Run every 1 second
  }
}

// Task 2: Server Communication
void ServerTask(void *pvParameters) {
  while (1) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Sending Temperature: %.2f °C\n", Input);

      String eventDate = getFormattedTime();

      HTTPClient http;
      WiFiClient client;
      http.begin(client, serverName);

      DynamicJsonDocument jsonDoc(256);
      JsonObject root = jsonDoc.to<JsonObject>();
      root["enclosure_id"] = 113;
      JsonArray values = root.createNestedArray("values");

      JsonObject tempValueObj = values.createNestedObject();
      tempValueObj["key"] = "Temperature";
      tempValueObj["value"] = Input; // Send actual temperature data
      tempValueObj["uom"] = "°C";
      tempValueObj["event_date"] = eventDate;

      String jsonString;
      serializeJson(root, jsonString);
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST(jsonString);

      Serial.printf("HTTP Response code: %d\n", httpResponseCode);
      http.end();
    }

    Serial.println("Server Task Completed - Waiting for next interval");

    // Wait before sending the next data
    vTaskDelay(pdMS_TO_TICKS(5000)); // Send every 5 seconds
  }
}

// Task 3: Door Sensor Control
void DoorTask(void *pvParameters) {
  while (1) {
    int doorState = digitalRead(doorSensorPin);
    if (doorState == LOW) {
      digitalWrite(doorRelayPin, HIGH); // Door open, activate relay
      Serial.println("Door open: Relay ON");
    } else {
      digitalWrite(doorRelayPin, LOW);  // Door closed, deactivate relay
      Serial.println("Door closed: Relay OFF");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));  // Check every 2 seconds
  }
}

// Task 4: Stepper Motor Control
void StepperTask(void *pvParameters) {
  while (1) {
    Serial.println("Stepper task");
    // Clockwise rotation
    digitalWrite(DIR, LOW);
    for (int i = 0; i < steps_per_rev; i++) {
      digitalWrite(STEP, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(STEP, LOW);
      delayMicroseconds(stepDelay);
    }
    Serial.println("Stepper Clockwise");

    // Calculate and print RMS
    stepperRMS = sqrt((steps_per_rev * stepDelay) / 50000.0);
    Serial.printf("RMS: %.2f\n", stepperRMS);

    // Wait before the next cycle
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Task 5: Humidity Control with Random Values (for now)
void HumidityTask(void *pvParameters) {
  while (1) {
    int randomHumidity = random(30, 70);  // Generate a random humidity value
    Serial.printf("Random Humidity: %d%%\n", randomHumidity);

    // Send humidity to Nextion safely
    if (xSemaphoreTake(nextionMutex, pdMS_TO_TICKS(100))) {
      bool success = t1.setText(String(randomHumidity).c_str());
      if (success) {
        Serial.println("Humidity updated on display");
      } else {
        Serial.println("Failed to update humidity on display");
      }
      xSemaphoreGive(nextionMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // Run every 1 second
  }
}

// Helper function: Get formatted time from NTP client
String getFormattedTime() {
  timeClient.update();
  return timeClient.getFormattedTime();
}

// Helper function: Update NeoPixel LED color based on temperature
void updateLEDColor(double temperature) {
  if (temperature < 25) {
    strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue for cold
  } else if (temperature < 35) {
    strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green for moderate
  } else {
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red for hot
  }
  strip.show();
}
