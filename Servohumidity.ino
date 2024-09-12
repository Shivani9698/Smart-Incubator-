#include <ESP32Servo.h>
#include <DHT.h>

#define DHTPIN 4          // Define the pin where the DHT22 is connected
#define DHTTYPE DHT22     // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

Servo myservo;  // Create servo object to control a servo

int closedPosition = 0;   // Adjust this value according to your door's closed position
int openPosition = 120;    // Adjust this value according to your door's fully open position
bool doorOpen = false;     // To track the door's state

void setup() {
  myservo.attach(15);      // Attach the servo on pin 15 to the servo object
  Serial.begin(115200);    // Initialize serial communication
  dht.begin();             // Start the DHT22 sensor
  myservo.write(closedPosition);  // Ensure the door starts in the closed position
  delay(500);  // Stabilize motor
}

void loop() {
  // Read humidity from DHT22
  float humidity = dht.readHumidity();

  // Check if any reads failed
  if (isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Current Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  // Open the door if humidity is below 55%
  if (humidity < 55 && !doorOpen) {
    Serial.println("Humidity below 55%, opening door.");
    for (int pos = openPosition; pos >= closedPosition; pos--) {
      myservo.write(pos);  // Incrementally increase angle to open door
      delay(10);           // Adjust delay for smoother movement
    }
    doorOpen = true;   // Mark the door as open
  }

  // Close the door if humidity is above 90%
  if (humidity >= 80 && doorOpen) {
    Serial.println("Humidity above 80%, closing door.");
    for (int pos = closedPosition; pos <= openPosition; pos++)
     {
      myservo.write(pos);  // Incrementally decrease angle to close door
      delay(10);           // Adjust delay for smoother movement
    }
    doorOpen = false;  // Mark the door as closed
  }

  delay(2000);  // Wait for 2 seconds before checking again
}
