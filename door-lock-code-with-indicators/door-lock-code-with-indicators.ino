#include <ESP8266WiFi.h>
#include <SinricPro.h>
#include <SinricProLock.h>
#include <SinricProSwitch.h>
#include <Servo.h>

const char* ssid = "eee";
const char* password = "eee";
#define APP_KEY "eee"
#define APP_SECRET "eee"
#define DEVICE_ID "eee"

const int buttonPin = 12; // GPIO pin connected to the button (D6)
boolean buttonPressed = false;
int statusLedPin = 2;
Servo servoMotor;  // Create servo object to control a servo
int servoPin = 14; // GPIO pin connected to the servo (replace D5 with 14)

unsigned long previousMillis = 0; // Store the last time LED was updated
const long ledOnInterval = 500; // LED on interval (1 second)
const long ledOffInterval = 5000; // LED off interval (5 seconds)
bool ledState = LOW; // Track the current state of the LED
const long reconnectInterval = 10000; // Reconnect interval when trying to reconnect (10 seconds)
unsigned long lastReconnectAttempt = 0;

bool onPowerState(const String &deviceId, bool &state) {
  Serial.println("Received command from Sinric Pro");
  Serial.printf("Device ID: %s, State: %s\n", deviceId.c_str(), state ? "unlock" : "lock");
  if (deviceId == DEVICE_ID) {
    if (state) {
      Serial.println("Unlocking the smart lock");
      servoMotor.attach(servoPin);
      servoMotor.write(130); // Move servo to 130 degrees to unlock (adjust as needed)
      delay(500); // Wait for the servo to reach the position
      servoMotor.detach();
    } else {
      Serial.println("Locking the smart lock");
      servoMotor.attach(servoPin);
      servoMotor.write(15); // Move servo to 10 degrees to lock (adjust as needed)
      delay(500); // Wait for the servo to reach the position
      servoMotor.detach();
    }
  }
  return true; // Request handled properly
}

void onWiFiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("WiFi disconnected, attempting to reconnect...");
  lastReconnectAttempt = millis();
}

void setup() {
  Serial.begin(115200);
  pinMode(statusLedPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Register WiFi event handlers
  WiFi.onStationModeDisconnected(onWiFiDisconnect);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= reconnectInterval / 10) {
      previousMillis = currentMillis;
      ledState = !ledState; // Toggle the LED state
      digitalWrite(statusLedPin, ledState);
    }
  }

  Serial.println("");
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize Sinric Pro
  SinricProSwitch &mySwitch = SinricPro[DEVICE_ID];
  mySwitch.onPowerState(onPowerState);

  SinricPro.begin(APP_KEY, APP_SECRET);
}

void loop() {
  SinricPro.handle();

  int buttonState = digitalRead(buttonPin); // Read current button state

  // Check for button press transition (LOW to HIGH)
  if (!buttonPressed && buttonState == HIGH) {
    buttonPressed = true;  // Update flag for next loop
    Serial.println("Button pressed");  // Print "Button pressed" only once
  } else if (buttonState == LOW) {
    buttonPressed = false; // Reset the flag when button is released
  }

  // Check WiFi connection status
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    // Attempt to reconnect if the reconnect interval has passed
    if (currentMillis - lastReconnectAttempt >= reconnectInterval) {
      lastReconnectAttempt = currentMillis;
      Serial.print("Reconnecting to WiFi...");
      WiFi.begin(ssid, password);
    }

    // Blink the status LED every 1 second while trying to reconnect
    if (currentMillis - previousMillis >= reconnectInterval / 10) {
      previousMillis = currentMillis;
      ledState = !ledState; // Toggle the LED state
      digitalWrite(statusLedPin, ledState);
    }
  } else {
    // Blink the status LED with 1 second ON and 5 seconds OFF
    unsigned long currentMillis = millis();
    if (ledState == LOW && currentMillis - previousMillis >= ledOffInterval) {
      previousMillis = currentMillis;
      ledState = HIGH;
      digitalWrite(statusLedPin, LOW); // Turn the LED on
    } else if (ledState == HIGH && currentMillis - previousMillis >= ledOnInterval) {
      previousMillis = currentMillis;
      ledState = LOW;
      digitalWrite(statusLedPin, HIGH); // Turn the LED off
    }
  }
}
