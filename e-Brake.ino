#include <EEPROM.h>
#include <avr/wdt.h> // Watchdog Timer library

// --- Pin Definitions ---
const int CLAMP_BUTTON_PIN = 2;    // D2 - Falling edge interrupt (physical CLAMP button)
const int RELEASE_BUTTON_PIN = 3;  // D3 - Falling edge interrupt (physical RELEASE button)
const int BRAKE_PEDAL_PIN = 9;     // D9 - Brake pedal input (HIGH when depressed, LOW when released)

// --- UPDATED RELAY PIN ASSIGNMENTS ---
// Clamping relays are D11 and D5
const int MOTOR1_CLAMP_RELAY_PIN = 5;  // D5 - Physically clamps motor 1
const int MOTOR1_RELEASE_RELAY_PIN = 6; // D6 - Physically releases motor 1
const int MOTOR2_CLAMP_RELAY_PIN = 11; // D11 - Physically clamps motor 2
const int MOTOR2_RELEASE_RELAY_PIN = 12; // D12 - Physically releases motor 2

const int STATUS_LED_PIN = 7;      // D7 - Via FET

// --- Current sensor positions ---
const int MOTOR1_CURRENT_SENSOR_PIN = A7;
const int MOTOR2_CURRENT_SENSOR_PIN = A5;

// --- EEPROM Address ---
const int EEPROM_STATE_ADDRESS = 0;

// --- System States ---
enum SystemState {
  RELEASED,
  CLAMPED,
  CLMP_IN_PROGRESS,
  REL_IN_PROGRESS
};
SystemState currentSystemState;

// --- Motor Control Flags ---
volatile bool clampRequested = false;
volatile bool releaseRequested = false;

// --- FEATURE: Flag to ignore brake pedal check ---
bool ignoreBrakePedal = false; // Set to 'true' to ignore the brake pedal for testing.

// --- Serial Debugging Flag ---
const bool ENABLE_SERIAL_DEBUG = true; // Set to 'true' to enable detailed serial debugging output.

// --- Current Sensor Calibration (FOR ACS712 30A SENSOR) ---
const int CURRENT_QUIESCENT_ADC = 512;
// MODIFIED: Increased threshold to help prevent p5remature shutdowns due to noise.
const int CURRENT_THRESHOLD_ADC_DIFF = 135; // Adjusted from 170 to 250 (experiment with this value!)

// --- Moving Average for Current Sensing ---
// INCREASED SAMPLES FOR MORE SMOOTHING to reduce noise impact.
const int MOVING_AVERAGE_SAMPLES = 10; // Changed from 5 to 10
int motor1CurrentReadings[MOVING_AVERAGE_SAMPLES];
int motor2CurrentReadings[MOVING_AVERAGE_SAMPLES];
int motor1ReadIndex = 0;
int motor2ReadIndex = 0;
long motor1CurrentSum = 0;
long motor2CurrentSum = 0;

// --- Timers ---
const unsigned long CLAMP_OPERATION_TIMEOUT_MS = 10000; // 10 seconds for clamping
const unsigned long RELEASE_OPERATION_TIMEOUT_MS = 2000; // 2 seconds for releasing

const unsigned long LED_BLINK_PROGRESS_MS = 300;
const unsigned long SERIAL_REPORT_INTERVAL_MAIN_MS = 100;
const unsigned long SERIAL_REPORT_INTERVAL_CONTROL_MS = 50; // More frequent during motor ops

unsigned long lastLedToggleTime = 0;
bool ledState = false; // For blinking LED

// **GLOBAL DECLARATIONS FOR TIMER VARIABLES**
unsigned long lastSerialReportTimeMain = 0;
unsigned long lastSerialReportTimeControl = 0;


// --- Function Prototypes ---
// Declare all functions before setup() and loop() so the compiler knows about them.
void clampButtonPressed();
void releaseButtonPressed();
bool isBrakePedalPressed();
void setSystemState(SystemState newState);
const __FlashStringHelper* getStateName(SystemState state);
int readCurrent(int sensorPin, int readings[], int& index, long& sum);
void turnOffAllRelays();
void controlMotors(bool clampMode);
void updateLedStatus();


void setup() {
  Serial.begin(115200);

  if (ENABLE_SERIAL_DEBUG) {
    Serial.println(F("Arduino Nano Booting..."));
  }

  // Pin Modes Configuration
  pinMode(CLAMP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELEASE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PEDAL_PIN, INPUT);
  pinMode(MOTOR1_CLAMP_RELAY_PIN, OUTPUT);
  pinMode(MOTOR1_RELEASE_RELAY_PIN, OUTPUT);
  pinMode(MOTOR2_CLAMP_RELAY_PIN, OUTPUT);
  pinMode(MOTOR2_RELEASE_RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  // Ensure all relays are off at startup for safety
  turnOffAllRelays();

  // Attach interrupts for button presses (falling edge means button pressed)
  // Logic SWAPPED: CLAMP_BUTTON (D2) triggers RELEASE action
  //                RELEASE_BUTTON (D3) triggers CLAMP action
  attachInterrupt(digitalPinToInterrupt(CLAMP_BUTTON_PIN), releaseButtonPressed, FALLING); // Physical CLAMP button triggers RELEASE
  attachInterrupt(digitalPinToInterrupt(RELEASE_BUTTON_PIN), clampButtonPressed, FALLING); // Physical RELEASE button triggers CLAMP

  // Initialize moving average arrays
  for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
    motor1CurrentReadings[i] = CURRENT_QUIESCENT_ADC;
    motor2CurrentReadings[i] = CURRENT_QUIESCENT_ADC;
    motor1CurrentSum += CURRENT_QUIESCENT_ADC;
    motor2CurrentSum += CURRENT_QUIESCENT_ADC;
  }

  // Read previous system state from EEPROM
  currentSystemState = (SystemState)EEPROM.read(EEPROM_STATE_ADDRESS);
  // Validate EEPROM data; if invalid, default to RELEASED state
  if (currentSystemState != CLAMPED && currentSystemState != RELEASED &&
      currentSystemState != CLMP_IN_PROGRESS && currentSystemState != REL_IN_PROGRESS) {
    currentSystemState = RELEASED;
    EEPROM.update(EEPROM_STATE_ADDRESS, RELEASED);
  }
  Serial.print(F("Initial System State from EEPROM: "));
  Serial.println(getStateName(currentSystemState));

  updateLedStatus();

  wdt_enable(WDTO_1S); // Enable Watchdog Timer with 1-second timeout
  if (ENABLE_SERIAL_DEBUG) {
    Serial.println(F("Watchdog Timer Enabled."));
  }

  if (ENABLE_SERIAL_DEBUG) {
    Serial.print(F("Brake pedal check is currently "));
    Serial.println(ignoreBrakePedal ? F("IGNORED (TEST MODE).") : F("ACTIVE (BRAKE REQUIRED)."));
    Serial.println(F("\n--- Current Sensor Calibration Info ---"));
    Serial.print(F("CURRENT_QUIESCENT_ADC (Target for 0A, 2.5V): ")); Serial.println(CURRENT_QUIESCENT_ADC);
    Serial.print(F("CURRENT_THRESHOLD_ADC_DIFF (Target for Stall Current, e.g., 15A): ")); Serial.println(CURRENT_THRESHOLD_ADC_DIFF);
    Serial.println(F("Monitor 'M_RAW_ADC' & 'M_AVG_ADC' values during operation to fine-tune THRESHOLD."));
    Serial.println(F("-------------------------------------\n"));
  }
}


void loop() {
  wdt_reset(); // Reset the watchdog timer to prevent system reset

  updateLedStatus(); // Update LED status based on current system state

  // --- Process Clamp Request (triggered by physical RELEASE button) ---
  if (clampRequested) {
    if (ENABLE_SERIAL_DEBUG) {
      Serial.println(F("Clamp action requested (by physical RELEASE button)."));
    }
    // Clear the flag immediately, so if controlMotors takes time,
    // a new interrupt won't re-trigger it right after completion.
    clampRequested = false;

    // Only initiate clamping if currently RELEASED and not already in progress
    if (currentSystemState == RELEASED) {
      if (ENABLE_SERIAL_DEBUG) {
        Serial.println(F("Initiating Clamping Process..."));
      }
      setSystemState(CLMP_IN_PROGRESS);
      controlMotors(true); // This is a blocking call (waits for motors to stall or timeout)
      // After controlMotors returns, the state will be CLAMPED.
    } else if (currentSystemState == CLAMPED) {
      if (ENABLE_SERIAL_DEBUG) {
        Serial.println(F("Already CLAMPED. Skipping clamp operation."));
      }
    } else { // CLMP_IN_PROGRESS or REL_IN_PROGRESS
        if (ENABLE_SERIAL_DEBUG) {
            Serial.println(F("Operation in progress. Cannot initiate new clamp."));
        }
    }
    // Return early to ensure the state has settled before checking other flags or printing.
    return;
  }

  // --- Process Release Request (triggered by physical CLAMP button) ---
  if (releaseRequested) {
    if (ENABLE_SERIAL_DEBUG) {
      Serial.println(F("Release action requested (by physical CLAMP button)."));
    }
    // Clear the flag immediately
    releaseRequested = false;

    // Only initiate releasing if currently CLAMPED and not already in progress
    if (currentSystemState == CLAMPED) {
      if (ENABLE_SERIAL_DEBUG) {
        Serial.println(F("Initiating Releasing Process..."));
      }
      setSystemState(REL_IN_PROGRESS);
      controlMotors(false); // This is a blocking call (waits for timeout)
      // After controlMotors returns, the state will be RELEASED.
    } else if (currentSystemState == RELEASED) {
      if (ENABLE_SERIAL_DEBUG) {
        Serial.println(F("Already RELEASED. Skipping release operation."));
      }
    } else { // CLMP_IN_PROGRESS or REL_IN_PROGRESS
        if (ENABLE_SERIAL_DEBUG) {
            Serial.println(F("Operation in progress. Cannot initiate new release."));
        }
    }
    // Return early to ensure the state has settled.
    return;
  }

  // --- Serial Reporting of State and Continuous Current Values (every 100ms) ---
  // This block runs regardless of the system state, providing continuous monitoring.
  unsigned long currentTime = millis();
  if (ENABLE_SERIAL_DEBUG && (currentTime - lastSerialReportTimeMain >= SERIAL_REPORT_INTERVAL_MAIN_MS)) {
    lastSerialReportTimeMain = currentTime;

    Serial.print(F("State: "));
    Serial.print(getStateName(currentSystemState));

    // Read current values even when idle for continuous monitoring
    int motor1RawADC_main = analogRead(MOTOR1_CURRENT_SENSOR_PIN);
    int motor2RawADC_main = analogRead(MOTOR2_CURRENT_SENSOR_PIN);
    // Note: The 'readCurrent' function modifies the moving average buffer.
    // For idle states, these will just reflect the quiescent level.
    int motor1AvgADC_main = readCurrent(MOTOR1_CURRENT_SENSOR_PIN, motor1CurrentReadings, motor1ReadIndex, motor1CurrentSum);
    int motor2AvgADC_main = readCurrent(MOTOR2_CURRENT_SENSOR_PIN, motor2CurrentReadings, motor2ReadIndex, motor2CurrentSum);

    Serial.print(F(" -- M1_RAW_ADC: ")); Serial.print(motor1RawADC_main);
    Serial.print(F(", M2_RAW_ADC: ")); Serial.print(motor2RawADC_main);
    Serial.print(F(" -- M1_AVG_ADC: ")); Serial.print(motor1AvgADC_main);
    Serial.print(F(", M2_AVG_ADC: ")); Serial.print(motor2AvgADC_main);
    Serial.println(); // Always end with a new line
  }
}


// --- Interrupt Service Routines (ISRs) ---
// These functions are called when their respective button pins go LOW (falling edge).
// They simply set a flag, and the main loop processes the request.
// This keeps ISRs short and avoids potential issues with Serial.print or delays inside ISRs.

// physical CLAMP_BUTTON (D2) now triggers RELEASE action
void releaseButtonPressed() { // This ISR runs when the physical CLAMP button (D2) is pressed
  // Allow release if brake pressed (or ignored for test) and no other operation is in progress
  if ((isBrakePedalPressed() || ignoreBrakePedal) && currentSystemState != CLMP_IN_PROGRESS && currentSystemState != REL_IN_PROGRESS) {
    if (currentSystemState == CLAMPED) {
      releaseRequested = true; // Request a RELEASE action
    }
  }
}

// physical RELEASE_BUTTON (D3) now triggers CLAMP action
void clampButtonPressed() { // This ISR runs when the physical RELEASE button (D3) is pressed
  // Allow clamp if brake pressed (or ignored for test) and no other operation is in progress
  if ((isBrakePedalPressed() || ignoreBrakePedal) && currentSystemState != CLMP_IN_PROGRESS && currentSystemState != REL_IN_PROGRESS) {
    if (currentSystemState == RELEASED) {
      clampRequested = true; // Request a CLAMP action
    }
  }
}


// --- Helper Functions ---

bool isBrakePedalPressed() {
  return digitalRead(BRAKE_PEDAL_PIN) == HIGH;
}

// Returns a descriptive string for the current system state, using F-macro for Flash memory.
const __FlashStringHelper* getStateName(SystemState state) {
  switch (state) {
    case RELEASED: return F("RELEASED");
    case CLAMPED: return F("CLAMPED");
    case CLMP_IN_PROGRESS: return F("CLMP_IN_PROGRESS");
    case REL_IN_PROGRESS: return F("REL_IN_PROGRESS");
    default: return F("UNKNOWN");
  }
}

// Sets the system state and updates the serial monitor and LED.
void setSystemState(SystemState newState) {
  currentSystemState = newState;
  Serial.print(F("System State changed to: "));
  Serial.println(getStateName(currentSystemState));
  updateLedStatus();
}

// Updates the status LED based on the current system state.
void updateLedStatus() {
  unsigned long currentTime = millis();
  switch (currentSystemState) {
    case CLAMPED:
      digitalWrite(STATUS_LED_PIN, HIGH); // Solid ON when CLAMPED
      break;
    case RELEASED:
      digitalWrite(STATUS_LED_PIN, LOW); // Solid OFF when RELEASED
      break;
    case CLMP_IN_PROGRESS:
    case REL_IN_PROGRESS:
      // Blinking during operation (progress)
      if (currentTime - lastLedToggleTime >= LED_BLINK_PROGRESS_MS) {
        lastLedToggleTime = currentTime;
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState);
      }
      break;
  }
}

// Reads current from a sensor, updates a moving average, and returns the new average.
int readCurrent(int sensorPin, int readings[], int& index, long& sum) {
  sum -= readings[index]; // Subtract the oldest reading
  readings[index] = analogRead(sensorPin); // Read the new value
  sum += readings[index]; // Add the new reading to the sum
  index = (index + 1) % MOVING_AVERAGE_SAMPLES; // Move to the next index (circular buffer)
  return sum / MOVING_AVERAGE_SAMPLES; // Return the new average
}

// Turns off all motor control relays for safety.
void turnOffAllRelays() {
  digitalWrite(MOTOR1_CLAMP_RELAY_PIN, LOW);
  digitalWrite(MOTOR1_RELEASE_RELAY_PIN, LOW);
  digitalWrite(MOTOR2_CLAMP_RELAY_PIN, LOW);
  digitalWrite(MOTOR2_RELEASE_RELAY_PIN, LOW);
  if (ENABLE_SERIAL_DEBUG) {
    Serial.println(F("All relays turned OFF."));
  }
}

// Controls the motor clamping or releasing operation. This is a blocking function.
void controlMotors(bool clampMode) {
  unsigned long startTime = millis();
  unsigned long operationTimeout = clampMode ? CLAMP_OPERATION_TIMEOUT_MS : RELEASE_OPERATION_TIMEOUT_MS;

  turnOffAllRelays(); // Ensure all motors are off before starting any new operation

  if (clampMode) {
    // --- CLAMPING MODE --- (Detect current spike for stall OR timeout)
    digitalWrite(MOTOR1_CLAMP_RELAY_PIN, HIGH);
    if (ENABLE_SERIAL_DEBUG) { Serial.println(F("Activating Motor 1 CLAMP relay...")); }
    _delay_ms(100); // Small delay to stagger motor start, reduces sudden current draw
    digitalWrite(MOTOR2_CLAMP_RELAY_PIN, HIGH);
    if (ENABLE_SERIAL_DEBUG) { Serial.println(F("Activating Motor 2 CLAMP relay...")); }

    bool motor1Clamped = false;
    bool motor2Clamped = false;

    // Loop until both motors have stalled or timeout occurs
    while (!motor1Clamped || !motor2Clamped) {
      wdt_reset(); // Keep the watchdog happy while in this loop
      updateLedStatus(); // Keep LED status updated (blinking during progress)

      // Check for operation timeout
      if (millis() - startTime >= operationTimeout) {
        Serial.println(F("Clamping operation timed out. Considering system CLAMPED."));
        turnOffAllRelays();
        setSystemState(CLAMPED); // Set state to CLAMPED even if timed out
        EEPROM.update(EEPROM_STATE_ADDRESS, CLAMPED);
        Serial.println(F("Clamping operation SUCCESSFUL (timeout reached). System CLAMPED."));
        return; // Exit the function, operation completed
      }

      // Monitor Motor 1 current
      int motor1RawReading = analogRead(MOTOR1_CURRENT_SENSOR_PIN); // Get instantaneous raw reading
      int motor1AvgCurrent = readCurrent(MOTOR1_CURRENT_SENSOR_PIN, motor1CurrentReadings, motor1ReadIndex, motor1CurrentSum); // Get new average

      if (!motor1Clamped) { // Only check if motor 1 is still clamping
        // Check if the averaged current exceeds the threshold (indicating stall)
        if (abs(motor1AvgCurrent - CURRENT_QUIESCENT_ADC) > CURRENT_THRESHOLD_ADC_DIFF) {
          if (ENABLE_SERIAL_DEBUG) {
            Serial.print(F("CONTROL_LOOP - Motor 1 current spike detected (AVG: ")); Serial.print(motor1AvgCurrent);
            Serial.print(F(", RAW: ")); Serial.print(motor1RawReading); Serial.println(F(")! Turning off Motor 1."));
          }
          _delay_ms(100); // Small delay to ensure relay response
          digitalWrite(MOTOR1_CLAMP_RELAY_PIN, LOW); // Turn off motor 1 clamp relay
          motor1Clamped = true; // Mark motor 1 as clamped
        }
      }

      // Monitor Motor 2 current (similar logic to Motor 1)
      int motor2RawReading = analogRead(MOTOR2_CURRENT_SENSOR_PIN);
      int motor2AvgCurrent = readCurrent(MOTOR2_CURRENT_SENSOR_PIN, motor2CurrentReadings, motor2ReadIndex, motor2CurrentSum);

      if (!motor2Clamped) {
        if (abs(motor2AvgCurrent - CURRENT_QUIESCENT_ADC) > CURRENT_THRESHOLD_ADC_DIFF) {
          if (ENABLE_SERIAL_DEBUG) {
            Serial.print(F("CONTROL_LOOP - Motor 2 current spike detected (AVG: ")); Serial.print(motor2AvgCurrent);
            Serial.print(F(", RAW: ")); Serial.print(motor2RawReading); Serial.println(F(")! Turning off Motor 2."));
          }
          _delay_ms(100);
          digitalWrite(MOTOR2_CLAMP_RELAY_PIN, LOW);
          motor2Clamped = true;
        }
      }

      // Serial print current values frequently during operation for debugging
      unsigned long currentControlTime = millis();
      if (ENABLE_SERIAL_DEBUG && (currentControlTime - lastSerialReportTimeControl >= SERIAL_REPORT_INTERVAL_CONTROL_MS)) {
        lastSerialReportTimeControl = currentControlTime;
        Serial.print(F("CONTROL_LOOP (Clamping) - M1_AVG: ")); Serial.print(motor1AvgCurrent);
        Serial.print(F(", M2_AVG: ")); Serial.println(motor2AvgCurrent);
      }
      _delay_ms(10); // Small delay to avoid busy-waiting and allow other tasks/ISR to run
    }

    // If we reach here, both motors were clamped successfully by current spike detection
    turnOffAllRelays(); // Ensure all relays are off
    setSystemState(CLAMPED); // Set final state
    EEPROM.update(EEPROM_STATE_ADDRESS, CLAMPED); // Save state to EEPROM
    Serial.println(F("Clamping operation SUCCESSFUL (current spike detected). System CLAMPED."));

  } else {
    // --- RELEASING MODE --- (Fixed 2-second timeout as SUCCESS, no current monitoring needed)
    digitalWrite(MOTOR1_RELEASE_RELAY_PIN, HIGH);
    if (ENABLE_SERIAL_DEBUG) { Serial.println(F("Activating Motor 1 RELEASE relay...")); }
    _delay_ms(100); // Stagger start
    digitalWrite(MOTOR2_RELEASE_RELAY_PIN, HIGH);
    if (ENABLE_SERIAL_DEBUG) { Serial.println(F("Activating Motor 2 RELEASE relay...")); }

    // Run motors for the defined RELEASE_OPERATION_TIMEOUT_MS duration
    while (millis() - startTime < operationTimeout) {
      wdt_reset(); // Keep watchdog happy
      updateLedStatus(); // Keep LED updated

      // Serial print progress
      unsigned long currentControlTime = millis();
      if (ENABLE_SERIAL_DEBUG && (currentControlTime - lastSerialReportTimeControl >= SERIAL_REPORT_INTERVAL_CONTROL_MS)) {
          lastSerialReportTimeControl = currentControlTime;
          Serial.print(F("CONTROL_LOOP (Releasing) - Elapsed: ")); Serial.print(currentControlTime - startTime); Serial.print(F("ms / ")); Serial.print(operationTimeout); Serial.println(F("ms"));
      }
      _delay_ms(10); // Small delay
    }

    // Timeout reached, consider release operation successful
    turnOffAllRelays(); // Turn off relays
    setSystemState(RELEASED); // Set final state
    EEPROM.update(EEPROM_STATE_ADDRESS, RELEASED); // Save state to EEPROM
    Serial.println(F("Releasing operation SUCCESSFUL (timeout reached). System RELEASED."));
  }
}
