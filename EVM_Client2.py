const int in1 = 7;
const int in2 = 8;
const int enA = 5;
const int buttonPin = 2;
const int ledPin = 13;
const int currentSensePin = A0;

bool direction = true;  // true = opening, false = closing
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// PWM & current threshold values
const int openPWM = 240;          // Increased for reliability
const int closePWM = 240;
const int torqueBoostPWM = 255;   // Full power boost
const int torqueBoostTime = 165;  // Boost duration in milliseconds

// Current detection tuning
const float emaAlpha = 0.20;
const float minRunningCurrent = 0.3;
const float openSpikeRate = 0.32;
const float closeSpikeRate = 0.32;
const unsigned long ignoreSpikeTime = 250;

float filteredCurrent = 0.0;
float previousFiltered = 0.0;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Filtered Current (A)");
}

void loop() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;

      if (currentButtonState == LOW) {
        runMotorUntilSpike();
      }
    }
  }

  lastButtonState = reading;

  float rawCurrent = readCurrent();
  filteredCurrent = emaAlpha * rawCurrent + (1 - emaAlpha) * filteredCurrent;
  Serial.println(filteredCurrent);
}

void runMotorUntilSpike() {
  digitalWrite(ledPin, HIGH);

  // Set motor direction
  if (direction) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  // Apply torque boost pulse
  analogWrite(enA, torqueBoostPWM);
  delay(torqueBoostTime);

  // Switch to regular operating PWM
  if (direction) {
    analogWrite(enA, openPWM);
  } else {
    analogWrite(enA, closePWM);
  }

  unsigned long startTime = millis();
  previousFiltered = readCurrent();
  filteredCurrent = previousFiltered;

  while (true) {
    delay(20);
    float current = readCurrent();
    float previous = filteredCurrent;
    filteredCurrent = emaAlpha * current + (1 - emaAlpha) * filteredCurrent;

    unsigned long elapsed = millis() - startTime;
    float rate = (filteredCurrent - previous) / 0.020;

    bool overMinCurrent = filteredCurrent > minRunningCurrent;
    bool timeOk = elapsed > ignoreSpikeTime;

    if (timeOk && overMinCurrent) {
      if (direction && rate > openSpikeRate) break;
      if (!direction && rate > closeSpikeRate) break;
    }

    Serial.print("Current: "); Serial.print(filteredCurrent);
    Serial.print(" | Rate: "); Serial.println(rate);
  }

  // Stop motor
  analogWrite(enA, 0);
  digitalWrite(ledPin, LOW);
  direction = !direction;  // Toggle direction
}

float readCurrent() {
  int sensorValue = analogRead(currentSensePin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float current = (voltage - 2.5) / 0.066;
  return abs(current);
}
