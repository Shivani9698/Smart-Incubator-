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

// Pin assignments for the temperature and relay
#define ONE_WIRE_BUS 4
#define RELAY_PIN 25
#define doorSensorPin 13 // Door sensor pin
#define doorRelayPin 27  // Door relay pin
#define DIR 2           // Stepper motor direction pin
#define STEP 5          // Stepper motor step pin
#define LED_PIN 12      // Pin for the addressable RGB LED
#define NUM_PIXELS 1    // Number of pixels in the strip

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

// Function prototypes for tasks
void TemperatureTask(void *pvParameters);
void ServerTask(void *pvParameters);
void DoorTask(void *pvParameters);
void StepperTask(void *pvParameters);
void updateLEDColor(double temperature);

Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

TaskHandle_t serverTaskHandle; // Task handle for ServerTask

void setup() {
  Serial.begin(115200);

  // Initialize the NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Initialize temperature sensor
  sensors.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetTunings(Kp, Ki, Kd);

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

  // Create FreeRTOS tasks
  xTaskCreate(TemperatureTask, "Temperature Control", 10000, NULL, 3, NULL);  // Task 1: Temperature Control
  xTaskCreate(ServerTask, "Server Task", 12000, NULL, 1, &serverTaskHandle);  // Task 2: Server Communication
  xTaskCreate(DoorTask, "Door Control", 10000, NULL, 0, NULL);                // Task 3: Door Sensor Control
  xTaskCreate(StepperTask, "Stepper Control", 8000, NULL, 2, NULL);           // Task 4: Stepper Motor Control
}

void loop() {
  // No code needed in the main loop since everything is handled by tasks
}

// Task 1: Temperature Control with PID
void TemperatureTask(void *pvParameters) {
  while (1) {
    Serial.println("Temp task");
    sensors.requestTemperatures();
    int validSensorCount = 0;
    Input = 0;

    // Loop through each temperature sensor
    for (int i = 0; i < sensors.getDeviceCount(); i++) {
      double temp = sensors.getTempCByIndex(i);
      if (temp != -127.00 && temp != 85.00) { // Valid temperature check
        Input += temp;
        validSensorCount++;
      }
    }

    if (validSensorCount > 0) {
      Input /= validSensorCount;
      Serial.print("Average Temperature: ");
      Serial.println(Input);

      myPID.Compute();  // Run PID computation

      if (Output > 100) {
        digitalWrite(RELAY_PIN, HIGH); // Heating
        Serial.println("Heating ON");
      } else {
        digitalWrite(RELAY_PIN, LOW);  // Heating OFF
        Serial.println("Heating OFF");
      }

      updateLEDColor(Input); // Update LED color based on temperature
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Run every 1 second
  }
}

// Task 2: Server Communication with Average Temperature (Executes Once)
void ServerTask(void *pvParameters) {
  Serial.println("Server Task Started - Send Data to Server");

  if (WiFi.status() == WL_CONNECTED) {
    // Send average temperature from PID control
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
    tempValueObj["value"] = random(1,100);
    tempValueObj["uom"] = "°C";
    tempValueObj["event_date"] = eventDate;

    String jsonString;
    serializeJson(root, jsonString);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonString);

    Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    http.end();
  }

  Serial.println("Server Task Completed - Deleting Task");

  // Delete the server task after it completes
  vTaskDelete(serverTaskHandle);
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

    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second

    // Counterclockwise rotation
    digitalWrite(DIR, HIGH);
    for (int i = 0; i < steps_per_rev; i++) {
      digitalWrite(STEP, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(STEP, LOW);
      delayMicroseconds(stepDelay);
    }
    Serial.println("Stepper Counter-Clockwise");

    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
  }
}

// Helper function to get the formatted time from the NTP client
String getFormattedTime() {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  char formattedTime[25];
  sprintf(formattedTime, "%04d-%02d-%02dT%02d:%02d:%02d", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
  return String(formattedTime);
}

// Update LED color based on temperature
void updateLEDColor(double temperature) {
  if (temperature >= 36.0 && temperature < 36.5) {
    // Blue LED
    strip.setPixelColor(0, strip.Color(0, 0, 255)); // RGB: Blue
  } else if (temperature >= 36.5 && temperature <= 37.0) {
    // Green LED
    strip.setPixelColor(0, strip.Color(0, 255, 0)); // RGB: Green
  } else if (temperature > 37.0) {
    // Red LED
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // RGB: Red
  }
  strip.show(); // Update the strip
}
