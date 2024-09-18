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

// Pin assignments
#define DIR 27
#define STEP 26
#define DOOR_SENSOR_PIN 13
#define DOOR_RELAY_PIN 32
#define ONE_WIRE_BUS 4
#define RELAY_PIN 25

const int steps_per_rev = 500000;
int a = 50000;

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
void printAddress(DeviceAddress deviceAddress);
String getFormattedTime();

void setup() {
    Serial.begin(115200);

    // WiFiManager initialization
    WiFiManager wm;
    bool res = wm.autoConnect("Incubator", "Lset@123");

    if (!res) {
        Serial.println("Failed to connect to WiFi");
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
    digitalWrite(DOOR_RELAY_PIN, LOW);

    // Initialize temperature sensors and relay
    sensors.begin();
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    // Initialize PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);

    // Create FreeRTOS tasks
    xTaskCreate(StepperMotorTask, "StepperMotorTask", 10000, NULL, 1, &StepperMotorTaskHandle);
    xTaskCreate(DoorSensorTask, "DoorSensorTask", 10000, NULL, 1, &DoorSensorTaskHandle);
    xTaskCreate(TempControlTask, "TempControlTask", 10000, NULL, 1, &TempControlTaskHandle);
    xTaskCreate(ApiTask, "ApiTask", 10000, NULL, 1, &ApiTaskHandle);  // New task for API handling
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

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
    }
}

void ApiTask(void *pvParameters) {
    while (true) {
        if (WiFi.status() == WL_CONNECTED) {
            // Fetch real-time date and time
            timeClient.update();
            String eventDate = getFormattedTime();  // Correct date and time

            // Log temperature value before sending it
            Serial.print("Preparing to send temperature data: ");
            Serial.println(Input);

            // Create JSON data for PID temperature
            HTTPClient http;
            WiFiClient client;

            http.begin(client, serverName);
            DynamicJsonDocument jsonDoc(256);
            JsonObject root = jsonDoc.to<JsonObject>();
            root["enclosure_id"] = 114;  // Make sure to use the correct enclosure ID

            JsonArray values = root.createNestedArray("values");

            JsonObject tempValueObj = values.createNestedObject();
            tempValueObj["key"] = "Temperature";
            tempValueObj["value"] = Input;  // Sending PID Input (average temperature)
            tempValueObj["uom"] = "Celsius";
            tempValueObj["event_date"] = eventDate;  // Set real-time event date

            // Serialize JSON to a string
            String jsonString;
            serializeJson(root, jsonString);
            Serial.println("Sending JSON Data: " + jsonString);

            // Send HTTP POST request with JSON data
            http.addHeader("Content-Type", "application/json");
            int httpResponseCode = http.POST(jsonString);

            // Log the response code
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
            Serial.println("WiFi Disconnected");
        }

        vTaskDelay(pdMS_TO_TICKS(3000));  // Delay for 3 seconds
    }
}

void printAddress(DeviceAddress deviceAddress) {
    for (uint8_t i = 0; i < 8; i++) {
        if (deviceAddress[i] < 16) Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
    }
    Serial.println();
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
