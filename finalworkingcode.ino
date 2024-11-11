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
double Setpoint = 37.5;  // Default setpoint (initial temp)
double Input, Output;
double Kp = 2, Ki = 5, Kd = 1;  // Initial PID values
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
double stepperRMS = 0; // RMS value for the stepper motord

// Function prototypes for taskste
void TemperatureTask(void *pvParameters);
void ServerTask(void *pvParameters);
void DoorTask(void *pvParameters);
void StepperTask(void *pvParameters);
void HumidityTask(void *pvParameters);
void updateLEDColor(double temperature);

// Nextion display
NexText t0 = NexText(1, 5, "t0"); // page 1, id 1 (temperature)
NexText t1 = NexText(1, 4, "t1"); // page 1, id 2 (humidity)

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
  xTaskCreate(TemperatureTask, "Temperature Control", 20000, NULL, 4, NULL);
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

  // Check if there's data available in the Serial buffer
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read input from Serial Monitor

    // Check if input starts with "temp" to set the temperature setpoint
    if (input.startsWith("temp")) {
      // Extract the temperature value after the command
      float newTemp = input.substring(5).toFloat();
      Setpoint = newTemp;  // Set the new temperature setpoint
      Serial.print("Temperature setpoint updated to: ");
      Serial.println(Setpoint);
    }

    // Check if input starts with "humi" to set the humidity
    if (input.startsWith("humi")) {
      // Extract the humidity value after the command
      float newHumi = input.substring(5).toFloat();
      // Set the new humidity value (You can add logic to control the humidity here)
      Serial.print("Humidity setpoint updated to: ");
      Serial.println(newHumi);
    }
  }
}


// Task 1: Temperature Control with DS18B20 Sensor
// Task 1: Temperature Control with DS18B20 Sensor
void TemperatureTask(void *pvParameters) {
  double previousSetpoint = Setpoint; // Variable to store the previous setpoint

  while (1) {
    sensors.requestTemperatures();
    double currentTemp = sensors.getTempCByIndex(0);
    Serial.printf("Temperature: %.2f °C\n", currentTemp);

    // Check if the setpoint has changed
    if (Setpoint != previousSetpoint) {
      // Update the previous setpoint to the new value
      previousSetpoint = Setpoint;
      
      // React quickly to the setpoint change by turning the heater on or off immediately
      if (currentTemp < Setpoint) {
        digitalWrite(RELAY_PIN, LOW); // Heating ON
        Serial.println("Immediate response: Heating ON due to new setpoint");
      } else if (currentTemp > Setpoint) {
        digitalWrite(RELAY_PIN, HIGH); // Heating OFF
        Serial.println("Immediate response: Heating OFF due to new setpoint");
      }
    }

    // Check the temperature with a tolerance range (fine control)
    double tolerance = 0.5;  // Adjust as needed for a smaller or larger range

    if (currentTemp < Setpoint - tolerance) {
      digitalWrite(RELAY_PIN, LOW); // Heating ON
      Serial.println("Heating ON");
    } else if (currentTemp > Setpoint + tolerance) {
      digitalWrite(RELAY_PIN, HIGH); // Heating OFF
      Serial.println("Heating OFF");
    }

    // Update PID based on current temperature
    Input = currentTemp;
    myPID.Compute();

    // Update LED color based on temperature
    updateLEDColor(Input);

    // Update the Nextion display with a timeout
    if (xSemaphoreTake(nextionMutex, pdMS_TO_TICKS(100))) {
      bool success = t0.setText(String(currentTemp).c_str());
      xSemaphoreGive(nextionMutex);
      if (success) {
        Serial.println("Temperature updated on display");
      } else {
        Serial.println("Failed to update temperature on display");
      }
    }

    // Reduce the delay to check the temperature more frequently
    vTaskDelay(pdMS_TO_TICKS(500)); // Check every 0.5 seconds
  }
}



// Function to update PID parameters based on the setpoint
void UpdatePIDParameters(double setpoint) {
  if (setpoint >= 25 && setpoint <= 30) {
    Kp = 2;
    Ki = 5;
    Kd = 1;
  } else if (setpoint > 30 && setpoint <= 35) {
    Kp = 3;
    Ki = 6;
    Kd = 1.5;
  } else if (setpoint > 35 && setpoint <= 40) {
    Kp = 4;
    Ki = 7;
    Kd = 2;
  } else {
    // Default values for unsupported setpoint ranges
    Kp = 2;
    Ki = 5;
    Kd = 1;
  }

  myPID.SetTunings(Kp, Ki, Kd);
  Serial.printf("PID Tunings Updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
}

// Task 3: Door Sensor Control
void DoorTask(void *pvParameters) {
  while (1) {
    int doorState = digitalRead(doorSensorPin);
    if (doorState == LOW) {
      digitalWrite(doorRelayPin, HIGH); // Door closed: activate relay
      Serial.println("Door closed: Relay OFF");
    } else {
      digitalWrite(doorRelayPin, LOW);  // Door open: deactivate relay
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

    // Counter-clockwise rotation
    digitalWrite(DIR, HIGH);
    for (int i = 0; i < steps_per_rev; i++) {
      digitalWrite(STEP, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(STEP, LOW);
      delayMicroseconds(stepDelay);
    }

    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay 10 seconds between rotations
  }
}

// Task 5: Humidity Control
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
  int red = 0, green = 0, blue = 0;
  
  if (temperature < Setpoint - 1) {
    // Temperature is below (Setpoint - 1), show blue
    blue = 25; 
  } else if (temperature > Setpoint + 1) {
    // Temperature is above (Setpoint + 1), show red
    red = 25; 
  } else {
    // Temperature is within (Setpoint - 1) to (Setpoint + 1), show green
    green = 25; 
  }

  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(red, green, blue));
  }
  strip.show(); // Update LED strip
}
