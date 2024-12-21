#include <ESP8266WiFi.h>
#include <SinricPro.h>
#include <SinricProLock.h>
#include <SinricProSwitch.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>

// RFID reader setup
#define SS_PIN 4
#define RST_PIN 0
MFRC522 mfrc522(SS_PIN, RST_PIN);

// wifi details
const char* ssid = "";
const char* password = "";
#define APP_KEY ""
#define APP_SECRET ""
#define DEVICE_ID ""


const int buttonPin = 5; // GPIO pin connected to the button (D6). Other wire of btn is to gnd
int statusLedPin = 2; // uses onboard LED to show wifi status
Servo servoMotor;  // Create servo object to control a servo
int servoPin = 16; // GPIO pin connected to the servo (D5)
int buttonState = 0; // Variable for reading the pushbutton status
int lastButtonState = HIGH;  // Previous state of the button
unsigned long lastDebounceTime = 0;  // The last time the output pin was toggled
unsigned long debounceDelay = 50;

unsigned long previousMillis = 0; // Store the last time LED was updated
const long ledOnInterval = 500; // LED on interval (1 second)
const long ledOffInterval = 5000; // LED off interval (5 seconds)
bool ledState = LOW; // Track the current state of the LED
const long reconnectInterval = 10000; // Reconnect interval when trying to reconnect (10 seconds)
unsigned long lastReconnectAttempt = 0;
bool lockState = false;

// Declare SinricProSwitch globally
SinricProSwitch &mySwitch = SinricPro[DEVICE_ID];




bool onPowerState(const String &deviceId, bool &state) {
  if (deviceId == DEVICE_ID) {
    if (state) {
      servoMotor.attach(servoPin);
      servoMotor.write(130); // Move servo to 130 degrees to unlock (adjust as needed)
      lockState = 0;
      delay(500); // Wait for the servo to reach the position
      servoMotor.detach();
    } else {
      servoMotor.attach(servoPin);
      servoMotor.write(15); // Move servo to 10 degrees to lock (adjust as needed)
      lockState = 1;
      delay(500); // Wait for the servo to reach the position
      servoMotor.detach();
    }
  }
  return true; 
}

void onWiFiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("WiFi disconnected, attempting to reconnect...");
  lastReconnectAttempt = millis();
}

void ManualSwitch() {
  if (lockState) {
          servoMotor.attach(servoPin);
          servoMotor.write(130); // Move servo to 130 degrees to unlock (adjust as needed)
          lockState = 0;
          delay(1000); // Wait for the servo to reach the position
          servoMotor.detach();
          
          // Update Sinric Pro state to unlocked
          mySwitch.sendPowerStateEvent(true);
        } else {
          servoMotor.attach(servoPin);
          servoMotor.write(15); // Move servo to 10 degrees to lock (adjust as needed)
          lockState = 1;
          delay(1000); // Wait for the servo to reach the position
          servoMotor.detach();
          
          // Update Sinric Pro state to locked
          mySwitch.sendPowerStateEvent(false);
    }
}

void setup() {
  // Serial.begin(115200);
  pinMode(statusLedPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Register WiFi event handlers
  WiFi.onStationModeDisconnected(onWiFiDisconnect);

  //card reader
  SPI.begin();    
  mfrc522.PCD_Init();    

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    // Serial.print(".");
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= reconnectInterval / 10) {
      previousMillis = currentMillis;
      ledState = !ledState; // Toggle the LED state
      digitalWrite(statusLedPin, ledState);
    }
  }

  // Serial.println("");
  // Serial.println("Connected to WiFi");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  // Initialize Sinric Pro
  mySwitch.onPowerState(onPowerState);
  SinricPro.begin(APP_KEY, APP_SECRET);
}

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if (!mfrc522.PICC_IsNewCardPresent()) {
    // No new card, continue with other code
  } else if (!mfrc522.PICC_ReadCardSerial()) {
    // Card read error, handle as needed
  } else {
    // Card read successfully
    // Simulate button press
   ManualSwitch();
    String content = "";
    byte letter;
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
      content.concat(String(mfrc522.uid.uidByte[i], HEX));
    }
   content.toUpperCase();
    // If you need to find the Serial number then add Line to print content.
    if (content == " A3 86 12 28"){
      ManualSwitch();
    }
    if (content == " A3 37 63 14"){
      ManualSwitch();
    }

  }


  //A3 37 63 14 Blue Tag
  //A3 86 12 28 White card

  //Handling button 
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        ManualSwitch();
      }
    }
  }

  lastButtonState = reading;

  SinricPro.handle();

  int buttonState = digitalRead(buttonPin); // Read current button state

  // Check WiFi connection status
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    // Attempt to reconnect if the reconnect interval has passed
    if (currentMillis - lastReconnectAttempt >= reconnectInterval) {
      lastReconnectAttempt = currentMillis;
      // Serial.print("Reconnecting to WiFi...");
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
