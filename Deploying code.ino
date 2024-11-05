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
#define RELAY_PIN 14
#define doorSensorPin 25
#define doorRelayPin 27
#define DIR 2
#define STEP 5
#define LED_PIN 13
#define NUM_PIXELS 55
#define DHT_PIN 26  

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
  //xTaskCreate(ServerTask, "Server Task", 15000, NULL, 1, &serverTaskHandle);
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

// Task 1: Temperature Control with DS18B20 Sensor
void TemperatureTask(void *pvParameters) {
  while (1) {
    sensors.requestTemperatures();
    double currentTemp = sensors.getTempCByIndex(0);  // Get temperature from DS18B20
    Serial.printf("Temperature: %.2f °C\n", currentTemp);
    
    // Simulate the PID control using the actual temperature
    Input = currentTemp;
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
      bool success = t0.setText(String(currentTemp).c_str());
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

// Task 3: Door Sensor Control
void DoorTask(void *pvParameters) {
  while (1) {
    int doorState = digitalRead(doorSensorPin);
    if (doorState == LOW) {
      digitalWrite(doorRelayPin, HIGH); // Door open, activate relay
      Serial.println("Door closed: Relay OFF");
    } else {
      digitalWrite(doorRelayPin, LOW);  // Door closed, deactivate relay
      Serial.println("Door open: Relay ON");
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
    // Anti-clockwise rotation
    digitalWrite(DIR, HIGH);
    for (int i = 0; i < steps_per_rev; i++) {
      digitalWrite(STEP, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(STEP, LOW);
      delayMicroseconds(stepDelay);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second between rotations
  }
}

// Task 5: Humidity Measurement with DHT22 Sensor
void HumidityTask(void *pvParameters) {
  while (1) {
    double humidity = dht.readHumidity();  // Read humidity from DHT22
    Serial.printf("Humidity: %.2f%%\n", humidity);

    // Send humidity to Nextion safely
    if (xSemaphoreTake(nextionMutex, pdMS_TO_TICKS(100))) {
      bool success = t1.setText(String(humidity).c_str());
      if (success) {
        Serial.println("Humidity updated on display");
      } else {
        Serial.println("Failed to update humidity on display");
      }
      xSemaphoreGive(nextionMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));  // Run every 1 second
  }
}

// Function to update LED color based on temperature
void updateLEDColor(double temperature) {
  if (temperature >= 36.0 && temperature < 36.5) {
    // Blue LED
    strip.setPixelColor(0, strip.Color(0, 0, 25)); // RGB: Blue
  } else if (temperature >= 36.5 && temperature <= 37.0) {
    // Green LED
    strip.setPixelColor(0, strip.Color(0, 25, 0)); // RGB: Green
  } else if (temperature > 37.0) {
    // Red LED
    strip.setPixelColor(0, strip.Color(25, 0, 0)); // RGB: Red
  }
  strip.show(); // Update the strip
}

// Helper function to get formatted time from NTP client
String getFormattedTime() {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();

  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", 
           (epochTime % 86400L) / 3600, 
           (epochTime % 3600) / 60, 
           (epochTime % 60));
  return String(buffer);
}

