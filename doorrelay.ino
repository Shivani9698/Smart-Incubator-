
// Define pin connections for door sensor and relays
const int doorSensorPin = 26; // Pin connected to the door sensor
const int doorRelayPin = 27;  // Pin connected to the relay controlling the door

void setup() {
  Serial.begin(115200);        // Start serial communication for debugging

  pinMode(doorSensorPin, INPUT_PULLUP); // Set door sensor pin as input with pull-up resistor
  pinMode(doorRelayPin, OUTPUT);        // Set door relay pin as output
  digitalWrite(doorRelayPin, LOW);      // Ensure door relay is off initially
  
}

void loop() {
  // Check the door sensor state
  int doorState = digitalRead(doorSensorPin); // Read the state of the door sensor
  
  if (doorState == LOW) {
    // Door is open (sensor circuit is closed)
    digitalWrite(doorRelayPin, HIGH); // Turn on door relay
    Serial.println("Door is open. Door Relay ON.");
  } else {
    // Door is closed (sensor circuit is open)
    digitalWrite(doorRelayPin, LOW); // Turn off door relay
    Serial.println("Door is closed. Door Relay OFF.");
  }

  
  delay(2000);  // Wait a few seconds between readings
}
