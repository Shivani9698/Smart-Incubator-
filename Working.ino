#include <Arduino.h>
#include <WiFiManager.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_task_wdt.h"  // Watchdog management

// Pin assignments
#define DIR 27
#define STEP 26
#define DOOR_SENSOR_PIN 13
#define DOOR_RELAY_PIN 32
#define ONE_WIRE_BUS 4
#define RELAY_PIN 25

const int steps_per_rev = 500000;
int a = 50000;  // Microseconds delay between steps

// Task Handles
TaskHandle_t StepperMotorTaskHandle = NULL;
TaskHandle_t DoorSensorTaskHandle = NULL;
TaskHandle_t TempControlTaskHandle = NULL;
TaskHandle_t ApiTaskHandle = NULL;

// PID variables
double Setpoint = 37.5;  // Desired temperature for incubation
double Input, Output;    // PID Input and Output
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // PID Controller

// OneWire instance and temperature sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
double temp[4];

// API settings
const char* serverName = "http://app.antzsystems.com/api/v1/iot/enclosure/metric/update";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000);  // Set IST timezone (UTC +5:30)

// Function Prototypes
void StepperMotorTask(void *pvParameters);
void DoorSensorTask(void *pvParameters);
void TempControlTask(void *pvParameters);
void ApiTask(void *pvParameters);
void checkWiFiConnection();
String getFormattedTime();

// Helper function to monitor heap memory
void monitorHeap() {
    Serial.print("Free heap: ");
    Serial.println(esp_get_free_heap_size());
    vTaskDelay(pdMS_TO_TICKS(10000));  // Check every 10 seconds
}

// Disable the task watchdog globally
void disableGlobalWatchdog() {
    // Deinitialize the Task Watchdog Timer (WDT) for all tasks
    esp_task_wdt_deinit();  // This will disable the task watchdog globally
}

void setup() {
    Serial.begin(115200);

    // Disable the task watchdog globally
    disableGlobalWatchdog();

    // WiFiManager initialization with timeout
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);  // Set timeout of 3 minutes
    bool res = wm.autoConnect("Incubator", "Lset@123");

    if (!res) {
        Serial.println("Failed to connect to WiFi within the timeout period.");
        ESP.restart();  // Restart ESP32 if WiFi connection fails
    } else {
        Serial.println("Connected to WiFi");
    }

    // Initialize NTP client for real-time updates
    timeClient.begin();

    // Initialize stepper motor pins
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);

    // Initialize door sensor and relay pins
    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
    pinMode(DOOR_RELAY_PIN, OUTPUT);
    digitalWrite(DOOR_RELAY_PIN, LOW);  // Relay initially off

    // Initialize temperature sensors and relay
    sensors.begin();
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    // Initialize PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);

    // Create FreeRTOS tasks with sufficient stack sizes
    xTaskCreatePinnedToCore(StepperMotorTask, "StepperMotorTask", 4096, NULL, 1, &StepperMotorTaskHandle, 0);  // Core 0
    xTaskCreatePinnedToCore(TempControlTask, "TempControlTask", 4096, NULL, 1, &TempControlTaskHandle, 1);   // Core 1
    xTaskCreatePinnedToCore(ApiTask, "ApiTask", 8192, NULL, 2, &ApiTaskHandle, 1);     
    xTaskCreatePinnedToCore(DoorSensorTask, "DoorSensorTask", 4096, NULL, 3, &DoorSensorTaskHandle, 0);  // Core 0, Priority 3
}

void loop() {
    monitorHeap();  // Monitor heap in the loop
}

// Stepper Motor Task
void StepperMotorTask(void *pvParameters) {
    while (true) {
        // Clockwise rotation
        digitalWrite(DIR, LOW);
        Serial.println("Spinning Clockwise...");
        for (int i = 0; i < steps_per_rev; i++) {
            digitalWrite(STEP, HIGH);
            delayMicroseconds(a);  // Small delay
            digitalWrite(STEP, LOW);
            delayMicroseconds(a);  // Small delay

            if (i % 50 == 0) {
                taskYIELD();  // Yield control to scheduler frequently
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Short delay between rotations

        // Counterclockwise rotation
        digitalWrite(DIR, HIGH);
        Serial.println("Spinning Anti-Clockwise...");
        for (int i = 0; i < steps_per_rev; i++) {
            digitalWrite(STEP, HIGH);
            delayMicroseconds(a);
            digitalWrite(STEP, LOW);
            delayMicroseconds(a);

            if (i % 50 == 0) {
                taskYIELD();  // Yield control to scheduler frequently
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Short delay between rotations
    }
}

// Temperature Control Task
void TempControlTask(void *pvParameters) {
    double hysteresis = 0.2; // Hysteresis buffer
    while (true) {
        sensors.requestTemperatures();

        // Read temperatures and calculate average
        Input = 0;
        int validSensorCount = 0;
        for (int i = 0; i < sensors.getDeviceCount(); i++) {
            temp[i] = sensors.getTempCByIndex(i);
            if (temp[i] != -127.00 && temp[i] != 85.00) {
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

            // Hysteresis for relay control
            if (Input >= Setpoint + hysteresis) {
                digitalWrite(RELAY_PIN, HIGH); // Turn off heating
                Serial.println("Heating OFF.");
            } else if (Input <= Setpoint - hysteresis) {
                digitalWrite(RELAY_PIN, LOW); // Turn on heating
                Serial.println("Heating ON.");
            }
        } else {
            Serial.println("No valid sensor readings. Skipping this cycle.");
        }

        vTaskDelay(1000);  // Yield to prevent watchdog timeout
    }
}

// API Task
void ApiTask(void *pvParameters) {
    // Declare HTTPClient and WiFiClient inside the task function
    HTTPClient http;
    WiFiClient client;

    while (true) {
        checkWiFiConnection();  // Ensure WiFi connection is stable

        if (WiFi.status() == WL_CONNECTED) {
            timeClient.update();
            String eventDate = getFormattedTime();  // Correct date and time

            Serial.print("Preparing to send temperature data: ");
            Serial.println(Input);

            http.begin(client, serverName);  // Reuse the same HTTPClient object
            DynamicJsonDocument jsonDoc(128);
            JsonObject root = jsonDoc.to<JsonObject>();
            root["enclosure_id"] = 114;

            JsonArray values = root.createNestedArray("values");

            JsonObject tempValueObj = values.createNestedObject();
            tempValueObj["key"] = "Temperature";
            tempValueObj["value"] = Input;
            tempValueObj["uom"] = "Celsius";
            tempValueObj["event_date"] = eventDate;

            String jsonString;
            serializeJson(root, jsonString);
            Serial.println("Sending JSON Data: " + jsonString);

            http.addHeader("Content-Type", "application/json");
            int httpResponseCode = http.POST(jsonString);

            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);

            if (httpResponseCode == 200) {
                String response = http.getString();
                Serial.println("Server response: " + response);
            } else {
                Serial.println("HTTP request failed");
            }

            http.end();
        } else {
            Serial.println("WiFi Disconnected.");
        }

        vTaskDelay(pdMS_TO_TICKS(60000));  // Delay for 60 seconds between requests
    }
}

// Door Sensor Task
void DoorSensorTask(void *pvParameters) {
    while (true) {
        int doorState = digitalRead(DOOR_SENSOR_PIN);  // Polling door sensor state
        if (doorState == LOW) {
            digitalWrite(DOOR_RELAY_PIN, HIGH);  // Door is closed
            Serial.println("Door is closed. Door Relay OFF.");
        } else {
            digitalWrite(DOOR_RELAY_PIN, LOW);   // Door is open
            Serial.println("Door is open. Door Relay ON.");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 2 seconds to check again
    }
}

// Helper function to get formatted time in ISO 8601 format (YYYY-MM-DDTHH:MM:SS)
String getFormattedTime() {
    unsigned long epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime((time_t *)&epochTime);

    char formattedTime[25];
    sprintf(formattedTime, "%04d-%02d-%02dT%02d:%02d:%02d",
            ptm->tm_year + 1900,
            ptm->tm_mon + 1,
            ptm->tm_mday,
            ptm->tm_hour,
            ptm->tm_min,
            ptm->tm_sec);
    return String(formattedTime);
}

// WiFi Reconnection Logic
void checkWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Disconnected. Trying to reconnect...");
        WiFi.reconnect();
        int retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < 5) {
            delay(1000);  // Wait 1 second before retrying
            retries++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("WiFi reconnected.");
        } else {
            Serial.println("Failed to reconnect. Restarting...");
            ESP.restart();  // Restart ESP if WiFi doesn't reconnect after retries
        }
    }
}
