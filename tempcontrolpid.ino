#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

// Pin assignments
#define ONE_WIRE_BUS 4    // Pin for DS18B20 sensors
#define RELAY_PIN 26      // Pin to control the relay

// Temperature threshold
double Setpoint = 37.5;  // Desired temperature (e.g., 37.5 °C for incubation)

// Define variables for PID control
double Input;            // Input for PID (average temperature)
double Output;           // PID output for controlling the relay
double Kp = 2, Ki = 5, Kd = 1;  // PID parameters (tune these)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // Initialize PID controller

// OneWire instance
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Variables for temperature input
double temp[4];  // To store temperature readings from 4 sensors

// Function to print the address of the sensors for debugging
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Start the DS18B20 temperature sensors
  sensors.begin();

  // Print the number of devices found
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount());
  Serial.println(" devices.");

  // Print addresses of the sensors
  DeviceAddress sensorAddress;
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    if (sensors.getAddress(sensorAddress, i)) {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" Address: ");
      printAddress(sensorAddress);
    } else {
      Serial.print("Unable to find address for sensor ");
      Serial.println(i + 1);
    }
  }

  // Set relay pin mode
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Initially off

  // Initialize PID control
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Adjust this for relay control

  // Set PID tuning parameters
  myPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
  // Request temperature from all sensors
  sensors.requestTemperatures();

  // Read temperature values from each sensor
  int validSensorCount = 0;
  Input = 0;
  
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    temp[i] = sensors.getTempCByIndex(i); // Read temperature from each sensor
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(temp[i]);

    // Only consider valid temperature readings
    if (temp[i] != -127.00 && temp[i] != 85.00) {  // -127 and 85°C are error codes
      Input += temp[i];
      validSensorCount++;
    }
  }

  // Calculate the average temperature if there are valid sensor readings
  if (validSensorCount > 0) {
    Input /= validSensorCount;  // Compute average temperature
    Serial.print("Average Temperature: ");
    Serial.println(Input);

    // Compute the PID output based on the average temperature
    myPID.Compute();

    // Relay control based on PID output
    // If the average temperature is below the setpoint, the relay will turn ON (heat)
    // If the average temperature is at or above the setpoint, the relay will turn OFF (stop heating)
    if (Output > 100) {  // You can adjust this threshold if needed
      digitalWrite(RELAY_PIN, HIGH);  // Turn on the relay (activate heating)
      Serial.println("Relay ON: Heating...");
    } else {
      digitalWrite(RELAY_PIN, LOW);   // Turn off the relay (stop heating)
      Serial.println("Relay OFF: Not Heating...");
    }
  } else {
    Serial.println("No valid sensor readings. Skipping this cycle.");
  }

  // Small delay to prevent overwhelming the sensors
  delay(1000);
}
