const int switchPin = 12; // D6 is GPIO14 on the D1 Mini
const int ledPin = 0;     // D4 is GPIO2 on the D1 Mini

bool doorWasOpen = false;
bool fadingOut = false;
unsigned long doorClosedTime = 0;
const unsigned long confirmTime = 2000; // 2 seconds confirmation time
const int fadeDurationOn = 1500; 
const int fadeDurationOff = 1500;     // Fade duration in milliseconds
int fadeStartValue = 0;
int fadeEndValue = 0;
int fadeDuration = 0;
bool fading = false;
unsigned long fadeStartTime = 0;
int currentLedValue = 0;

void setup() {
  // Initialize the switch pin as an input with internal pull-up resistor enabled
  pinMode(switchPin, INPUT_PULLUP);

  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Turn the LED off initially (inverted logic)
  analogWrite(ledPin, 255);
  currentLedValue = 255; // LED is off initially
}

void startFade(int startValue, int endValue, int duration) {
  fadeStartValue = startValue;
  fadeEndValue = endValue;
  fadeDuration = duration;
  fadeStartTime = millis();
  fading = true;
}

void updateFade() {
  if (fading) {
    unsigned long currentMillis = millis();
    unsigned long elapsed = currentMillis - fadeStartTime;

    if (elapsed >= fadeDuration) {
      analogWrite(ledPin, 255 - fadeEndValue); // Ensure the final value is set
      currentLedValue = fadeEndValue; // Update the current LED value
      fading = false; // End fading
    } else {
      int steps = abs(fadeEndValue - fadeStartValue);
      int currentStep = map(elapsed, 0, fadeDuration, 0, steps);
      int currentValue = fadeStartValue + (fadeEndValue > fadeStartValue ? currentStep : -currentStep);
      analogWrite(ledPin, 255 - currentValue); // Invert the value for transistor logic
      currentLedValue = currentValue; // Update the current LED value
    }
  }
}

void loop() {
// Read the state of the magnetic switch
  int switchState = digitalRead(switchPin);
  unsigned long currentMillis = millis();

  if (switchState == HIGH) { // Door is open
    if (!doorWasOpen) {
      startFade(currentLedValue, 255, fadeDurationOn); // Fade in the LED from current brightness to full (inverted logic)
      doorWasOpen = true; // Remember that the door was open
      fadingOut = false;  // Reset fading out flag
    }
  } else { // Door is closed
    
    if (doorWasOpen && !fadingOut) { // If the door was just closed and not fading out
      doorWasOpen = false;
      doorClosedTime = currentMillis; // Record the time the door was closed
      fadingOut = true; // Set fading out flag
    }
    
    // Check if 2 seconds have passed since the door was closed
    if (fadingOut && (currentMillis - doorClosedTime >= confirmTime)) {
      startFade(currentLedValue, 0, fadeDurationOff); // Fade out the LED from current brightness to off (inverted logic)
      fadingOut = false; // Reset fading out flag
    }
  }
  updateFade(); // Update the fade process
}
  
