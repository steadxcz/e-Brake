#include <EEPROM.h>

// Pins
const int relayClampM1     = 5;
const int relayReleaseM1   = 6;
const int relayClampM2     = 11;
const int relayReleaseM2   = 12;
const int clampButtonPin   = 2;
const int releaseButtonPin = 3;
const int brakePedalPin    = 9;
const int currentSensorM1  = A5;
const int currentSensorM2  = A7;
const int ledPin           = 7;

// Calibration
const int zeroCurrentADC_M1    = 517;
const int zeroCurrentADC_M2    = 515;
const int currentThresholdADC  = 270;
const int bufferSize           = 5;

// Blink intervals
const unsigned long blinkIntervalProcess = 350;
const unsigned long blinkIntervalFailure = 150;

// EEPROM address
const int eepromAddr = 0;

enum Command     { RELEASE = 0, CLAMP = 1 };
enum SystemState { IDLE, CLAMPING, RELEASING, CLAMPED, RELEASED, FAILED };

volatile bool clampRequested   = false;
volatile bool releaseRequested = false;
SystemState systemState        = IDLE;

// Debounce timers
unsigned long lastClampTime   = 0;
unsigned long lastReleaseTime = 0;

void setup() {
  analogReference(DEFAULT);
  Serial.begin(115200);

  pinMode(relayClampM1,   OUTPUT);
  pinMode(relayReleaseM1, OUTPUT);
  pinMode(relayClampM2,   OUTPUT);
  pinMode(relayReleaseM2, OUTPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(brakePedalPin,    INPUT);
  pinMode(clampButtonPin,   INPUT_PULLUP);
  pinMode(releaseButtonPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(clampButtonPin),
                  onClampInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(releaseButtonPin),
                  onReleaseInterrupt, FALLING);

  // Restore last known state
  byte lastCmd = EEPROM.read(eepromAddr);
  systemState  = (lastCmd == CLAMP) ? CLAMPED : RELEASED;
  Serial.println("System Ready");
}

void loop() {
  static unsigned long lastPrint     = 0;
  static unsigned long lastFailBlink = 0;
  static bool failBlinkState         = false;
  unsigned long now = millis();

  // Blink only in FAILED state
  if (systemState == FAILED) {
    if (now - lastFailBlink >= blinkIntervalFailure) {
      failBlinkState = !failBlinkState;
      digitalWrite(ledPin, failBlinkState);
      lastFailBlink = now;
    }
  }
  // Solid for clamped, off for released
  else if (systemState == CLAMPED) {
    digitalWrite(ledPin, HIGH);
  }
  else if (systemState == RELEASED) {
    digitalWrite(ledPin, LOW);
  }

  // Periodic serial status
  if (now - lastPrint >= 500) {
    int rawM1 = analogRead(currentSensorM1);
    int rawM2 = analogRead(currentSensorM2);
    Serial.print("ADC M1: "); Serial.print(rawM1);
    Serial.print(" | ADC M2: "); Serial.print(rawM2);
    Serial.print(" | State: ");  printState(systemState);
    Serial.println();
    lastPrint = now;
  }

  // Handle any pending requests
  if (clampRequested) {
    clampRequested = false;
    handleClamp();
  }
  if (releaseRequested) {
    releaseRequested = false;
    handleRelease();
  }
}

void onClampInterrupt() {
  unsigned long now = millis();
  if (now - lastClampTime > 100) {
    lastClampTime = now;
    if (digitalRead(brakePedalPin) == HIGH &&
        systemState != CLAMPING &&
        systemState != RELEASING) {
      clampRequested = true;
      Serial.println("Clamp requested");
    }
  }
}

void onReleaseInterrupt() {
  unsigned long now = millis();
  if (now - lastReleaseTime > 100) {
    lastReleaseTime = now;
    if (digitalRead(brakePedalPin) == HIGH &&
        systemState != CLAMPING &&
        systemState != RELEASING) {
      releaseRequested = true;
      Serial.println("Release requested");
    }
  }
}

void handleClamp() {
  if (EEPROM.read(eepromAddr) == CLAMP) {
    Serial.println("Already clamped. Skipping.");
    return;
  }

  Serial.println("Clamping...");
  systemState = CLAMPING;
  digitalWrite(relayClampM1, HIGH);
  digitalWrite(relayClampM2, HIGH);

  // Moving‐window and blink timer
  int readingsM1[bufferSize], readingsM2[bufferSize];
  for (int i = 0; i < bufferSize; i++) {
    readingsM1[i] = analogRead(currentSensorM1);
    readingsM2[i] = analogRead(currentSensorM2);
  }

  unsigned long start     = millis();
  unsigned long lastBlink = start;
  bool        blinkState  = false;

  int idx              = 0;
  bool m1Done          = false, m2Done = false;
  bool m1Triggered     = false, m2Triggered = false;
  unsigned long t1     = 0, t2 = 0;

  while ((millis() - start < 10000) && (!m1Done || !m2Done)) {
    // process‐blink at 350 ms
    if (millis() - lastBlink >= blinkIntervalProcess) {
      lastBlink = millis();
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState);
    }

    readingsM1[idx] = analogRead(currentSensorM1);
    readingsM2[idx] = analogRead(currentSensorM2);
    idx = (idx + 1) % bufferSize;

    int avgM1 = average(readingsM1, bufferSize);
    int avgM2 = average(readingsM2, bufferSize);

    if (!m1Triggered && abs(avgM1 - zeroCurrentADC_M1) >= currentThresholdADC) {
      m1Triggered = true; t1 = millis();
    }
    if (!m2Triggered && abs(avgM2 - zeroCurrentADC_M2) >= currentThresholdADC) {
      m2Triggered = true; t2 = millis();
    }

    if (m1Triggered && !m1Done && millis() - t1 >= 100) {
      digitalWrite(relayClampM1, LOW);
      m1Done = true;
      Serial.print("Motor 1 clamped @ "); Serial.println(avgM1);
    }
    if (m2Triggered && !m2Done && millis() - t2 >= 100) {
      digitalWrite(relayClampM2, LOW);
      m2Done = true;
      Serial.print("Motor 2 clamped @ "); Serial.println(avgM2);
    }

    delay(5);
  }

  if (!m1Done || !m2Done) {
    digitalWrite(relayClampM1, LOW);
    digitalWrite(relayClampM2, LOW);
    systemState = FAILED;
    Serial.println("Clamp FAILED (timeout)");
    return;
  }

  EEPROM.update(eepromAddr, CLAMP);
  systemState = CLAMPED;
  digitalWrite(ledPin, HIGH);
  Serial.println("Clamp complete");
}

void handleRelease() {
  if (EEPROM.read(eepromAddr) == RELEASE) {
    Serial.println("Already released. Skipping.");
    return;
  }

  Serial.println("Releasing...");
  systemState = RELEASING;
  digitalWrite(relayReleaseM1, HIGH);
  digitalWrite(relayReleaseM2, HIGH);

  int readingsM1[bufferSize], readingsM2[bufferSize];
  for (int i = 0; i < bufferSize; i++) {
    readingsM1[i] = analogRead(currentSensorM1);
    readingsM2[i] = analogRead(currentSensorM2);
  }

  unsigned long start     = millis();
  unsigned long lastBlink = start;
  bool        blinkState  = false;

  int idx              = 0;
  bool m1Done          = false, m2Done = false;
  bool m1Triggered     = false, m2Triggered = false;
  unsigned long t1     = 0, t2 = 0;

  while ((millis() - start < 10000) && (!m1Done || !m2Done)) {
    // process‐blink at 350 ms
    if (millis() - lastBlink >= blinkIntervalProcess) {
      lastBlink = millis();
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState);
    }

    readingsM1[idx] = analogRead(currentSensorM1);
    readingsM2[idx] = analogRead(currentSensorM2);
    idx = (idx + 1) % bufferSize;

    int avgM1 = average(readingsM1, bufferSize);
    int avgM2 = average(readingsM2, bufferSize);

    if (!m1Triggered && abs(avgM1 - zeroCurrentADC_M1) >= currentThresholdADC) {
      m1Triggered = true; t1 = millis();
    }
    if (!m2Triggered && abs(avgM2 - zeroCurrentADC_M2) >= currentThresholdADC) {
      m2Triggered = true; t2 = millis();
    }

    if (m1Triggered && !m1Done && millis() - t1 >= 100) {
      digitalWrite(relayReleaseM1, LOW);
      m1Done = true;
      Serial.print("Motor 1 released @ "); Serial.println(avgM1);
    }
    if (m2Triggered && !m2Done && millis() - t2 >= 100) {
      digitalWrite(relayReleaseM2, LOW);
      m2Done = true;
      Serial.print("Motor 2 released @ "); Serial.println(avgM2);
    }

    delay(5);
  }

  if (!m1Done || !m2Done) {
    digitalWrite(relayReleaseM1, LOW);
    digitalWrite(relayReleaseM2, LOW);
    systemState = FAILED;
    Serial.println("Release FAILED (timeout)");
    return;
  }

  EEPROM.update(eepromAddr, RELEASE);
  systemState = RELEASED;
  digitalWrite(ledPin, LOW);
  Serial.println("Release complete");
}

int average(int *arr, int len) {
  long sum = 0;
  for (int i = 0; i < len; i++) sum += arr[i];
  return sum / len;
}

void printState(SystemState s) {
  switch (s) {
    case IDLE:      Serial.print("IDLE");      break;
    case CLAMPING:  Serial.print("CLAMPING");  break;
    case RELEASING: Serial.print("RELEASING"); break;
    case CLAMPED:   Serial.print("CLAMPED");   break;
    case RELEASED:  Serial.print("RELEASED");  break;
    case FAILED:    Serial.print("FAILED");    break;
  }
}
