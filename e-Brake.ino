#include <EEPROM.h>
#include <avr/wdt.h> // Watchdog Timer library

// --- Pin Definitions ---
const int CLAMP_BUTTON_PIN = 2;   // D2 - Falling edge interrupt
const int RELEASE_BUTTON_PIN = 3;  // D3 - Falling edge interrupt
const int BRAKE_PEDAL_PIN = 9;    // D9 - Brake pedal input (HIGH when depressed, LOW when released)
const int MOTOR1_CLAMP_RELAY_PIN = 5;  // D5
const int MOTOR1_RELEASE_RELAY_PIN = 6; // D6
const int MOTOR2_CLAMP_RELAY_PIN = 11; // D11
const int MOTOR2_RELEASE_RELAY_PIN = 12; // D12
const int STATUS_LED_PIN = 7;     // D7 - Via FET

// --- FIXED: Current sensor positions swapped ---
// MOTOR1_CURRENT_SENSOR_PIN now reads from A7
const int MOTOR1_CURRENT_SENSOR_PIN = A7;
// MOTOR2_CURRENT_SENSOR_PIN now reads from A5
const int MOTOR2_CURRENT_SENSOR_PIN = A5;

// --- EEPROM Address ---
const int EEPROM_STATE_ADDRESS = 0;

// --- System States ---
enum SystemState {
  RELEASED,
  CLAMPED,
  CLMP_IN_PROGRESS,
  REL_IN_PROGRESS,
  FAILURE
};
SystemState currentSystemState;

// --- Motor Control Flags ---
volatile bool clampRequested = false;
volatile bool releaseRequested = false;

// --- FEATURE: Flag to ignore brake pedal check ---
// Set to 'true' to ignore the brake pedal and process button presses immediately.
// Set to 'false' to require the brake pedal to be pressed for operations.
bool ignoreBrakePedal = false; // Brake pedal check is ACTIVE. Change to 'true' if you need to ignore it for testing.

// --- NEW: Serial Debugging Flag ---
const bool ENABLE_SERIAL_DEBUG = true; // Set to 'true' to enable detailed serial debugging output.
                                       // Set to 'false' to disable most debugging prints.


// --- Current Sensor Calibration (FOR ACS712 30A SENSOR) ---
// IMPORTANT: CALIBRATE THESE VALUES FOR YOUR SENSOR AND MOTORS!
// ACS712 30A Sensitivity: 66 mV/A
// Arduino ADC: 10-bit (0-1023 steps), 5V reference (default AVCC).
// ADC steps per Volt = 1024 / 5V = 204.8 steps/V
// ADC steps per Amp = 66mV/A * (1V / 1000mV) * 204.8 = ~13.5 steps/A

// CURRENT_QUIESCENT_ADC: ADC reading when no current flows (VCC/2, typically 2.5V for 5V supply)
// 2.5V * 204.8 steps/V = 512 steps. Your test confirms this is working well!
const int CURRENT_QUIESCENT_ADC = 512;      // Based on your A5/A7 readings, this is good.

// CURRENT_THRESHOLD_ADC_DIFF: Approximate ADC difference for a desired current spike (e.g., 20A)
// For a ~20A spike: 20A * 13.5 steps/A = 270 steps.
// This is the magnitude of change from CURRENT_QUIESCENT_ADC that signifies a stall.
// You MUST ADJUST THIS value based on what you consider a "stall" for your specific motors.
const int CURRENT_THRESHOLD_ADC_DIFF = 270; // ADJUST THIS by testing your motors!
                                            // Start with a value and fine-tune by observing actual stall currents.

// --- Moving Average for Current Sensing ---
const int MOVING_AVERAGE_SAMPLES = 5; // Number of samples for smoothing current readings
int motor1CurrentReadings[MOVING_AVERAGE_SAMPLES];
int motor2CurrentReadings[MOVING_AVERAGE_SAMPLES];
int motor1ReadIndex = 0;
int motor2ReadIndex = 0;
long motor1CurrentSum = 0;
long motor2CurrentSum = 0;

// --- Timers ---
const unsigned long OPERATION_TIMEOUT_MS = 10000; // 10 seconds for clamp/release operation timeout
const unsigned long LED_BLINK_PROGRESS_MS = 300;  // 300 ms for LED blinking during operation
const unsigned long LED_BLINK_FAILURE_MS = 100;   // 150 ms for LED blinking on failure

// FEATURE: Serial print timer for main loop (100ms)
const unsigned long SERIAL_REPORT_INTERVAL_MAIN_MS = 100; // Report current and state every 100ms in main loop
unsigned long lastSerialReportTimeMain = 0;

// FEATURE: Serial print timer for controlMotors loop (more frequent)
const unsigned long SERIAL_REPORT_INTERVAL_CONTROL_MS = 50; // Report current every 50ms during motor ops
unsigned long lastSerialReportTimeControl = 0;


unsigned long lastLedToggleTime = 0;
bool ledState = false; // For blinking LED

// --- Function Prototypes ---
// Declare functions before setup/loop if they are called before their definition
void clampButtonPressed();
void releaseButtonPressed();
bool isBrakePedalPressed();
void setSystemState(SystemState newState);
String getStateName(SystemState state); // Helper to get state name for printing
int readCurrent(int sensorPin, int readings[], int& index, long& sum); // Reads and averages current
void turnOffAllRelays();
void controlMotors(bool clampMode); // Manages motor operation
void updateLedStatus();


void setup() {
  Serial.begin(115200); // Set baud rate to 115200

  if (ENABLE_SERIAL_DEBUG) { // Debug print
    Serial.println("Arduino Nano Booting...");
  }

  // Pin Modes Configuration
  pinMode(CLAMP_BUTTON_PIN, INPUT_PULLUP);    // Clamp button with internal pull-up
  pinMode(RELEASE_BUTTON_PIN, INPUT_PULLUP);  // Release button with internal pull-up
  pinMode(BRAKE_PEDAL_PIN, INPUT); // Brake pedal input
  pinMode(MOTOR1_CLAMP_RELAY_PIN, OUTPUT);
  pinMode(MOTOR1_RELEASE_RELAY_PIN, OUTPUT);
  pinMode(MOTOR2_CLAMP_RELAY_PIN, OUTPUT);
  pinMode(MOTOR2_RELEASE_RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  // Ensure all relays are off at startup for safety
  turnOffAllRelays(); // This function will respect ENABLE_SERIAL_DEBUG inside itself

  // Attach interrupts for button presses (falling edge means button pressed and pulled to GND)
  attachInterrupt(digitalPinToInterrupt(CLAMP_BUTTON_PIN), clampButtonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(RELEASE_BUTTON_PIN), releaseButtonPressed, FALLING);

  // Initialize moving average arrays with the quiescent ADC value (512 for 2.5V)
  for (int i = 0; i < MOVING_AVERAGE_SAMPLES; i++) {
    motor1CurrentReadings[i] = CURRENT_QUIESCENT_ADC;
    motor2CurrentReadings[i] = CURRENT_QUIESCENT_ADC;
    motor1CurrentSum += CURRENT_QUIESCENT_ADC;
    motor2CurrentSum += CURRENT_QUIESCENT_ADC;
  }

  // Read previous system state from EEPROM
  currentSystemState = (SystemState)EEPROM.read(EEPROM_STATE_ADDRESS);
  // Validate EEPROM data; if invalid, default to RELEASED state
  if (currentSystemState != CLAMPED && currentSystemState != RELEASED) {
    currentSystemState = RELEASED;
    EEPROM.update(EEPROM_STATE_ADDRESS, RELEASED); // Store default state in EEPROM
  }
  // This initial state print is important and not controlled by debug flag
  Serial.print("Initial System State from EEPROM: ");
  Serial.println(getStateName(currentSystemState));

  // Set initial LED status based on the retrieved state
  updateLedStatus();

  // Enable Watchdog Timer to reset if the program hangs for too long (~1 second)
  wdt_enable(WDTO_1S);
  if (ENABLE_SERIAL_DEBUG) { // Debug print
    Serial.println("Watchdog Timer Enabled.");
  }

  if (ENABLE_SERIAL_DEBUG) { // Debug print
    Serial.print("Brake pedal check is currently ");
    Serial.println(ignoreBrakePedal ? "IGNORED (TEST MODE)." : "ACTIVE (BRAKE REQUIRED).");
    Serial.println("\n--- Current Sensor Calibration Info ---");
    Serial.print("CURRENT_QUIESCENT_ADC (Target for 0A, 2.5V): "); Serial.println(CURRENT_QUIESCENT_ADC);
    Serial.print("CURRENT_THRESHOLD_ADC_DIFF (Target for Stall Current, e.g., 20A): "); Serial.println(CURRENT_THRESHOLD_ADC_DIFF);
    Serial.println("Monitor 'M_RAW_ADC' & 'M_AVG_ADC' values during operation to fine-tune THRESHOLD.");
    Serial.println("-------------------------------------\n");
  }
}


void loop() {
  wdt_reset(); // Reset the watchdog timer; must be called regularly to prevent reset

  updateLedStatus(); // Manage the status LED blinking/solid state

  // --- Process Clamp Request ---
  if (clampRequested) {
    if (ENABLE_SERIAL_DEBUG) { // Debug print
      Serial.println("Clamp requested.");
    }
    clampRequested = false; // Clear the flag immediately
    if (currentSystemState == CLAMPED) {
      if (ENABLE_SERIAL_DEBUG) { // Debug print
        Serial.println("Already CLAMPED. Skipping clamp operation.");
      }
    } else {
      if (ENABLE_SERIAL_DEBUG) { // Debug print
        Serial.println("Initiating Clamping Process...");
      }
      setSystemState(CLMP_IN_PROGRESS); // Change state to in-progress
      controlMotors(true); // Call function to manage motors for clamping
    }
  }

  // --- Process Release Request ---
  if (releaseRequested) {
    if (ENABLE_SERIAL_DEBUG) { // Debug print
      Serial.println("Release requested.");
    }
    releaseRequested = false; // Clear the flag immediately
    if (currentSystemState == RELEASED) {
      if (ENABLE_SERIAL_DEBUG) { // Debug print
        Serial.println("Already RELEASED. Skipping release operation.");
      }
    } else {
      if (ENABLE_SERIAL_DEBUG) { // Debug print
        Serial.println("Initiating Releasing Process...");
      }
      setSystemState(REL_IN_PROGRESS); // Change state to in-progress
      controlMotors(false); // Call function to manage motors for releasing
    }
  }

  // --- Serial Reporting of Current Values and State (every 100ms) from main loop ---
  unsigned long currentTime = millis();
  if (currentTime - lastSerialReportTimeMain >= SERIAL_REPORT_INTERVAL_MAIN_MS) {
    lastSerialReportTimeMain = currentTime;

    if (ENABLE_SERIAL_DEBUG) { // Debug prints
      // Read RAW current values directly from ADC for immediate debugging
      int motor1RawADC_main = analogRead(MOTOR1_CURRENT_SENSOR_PIN);
      int motor2RawADC_main = analogRead(MOTOR2_CURRENT_SENSOR_PIN);

      // Read averaged current values using the moving average function
      // Note: readCurrent updates the internal arrays and sums.
      int motor1AvgADC_main = readCurrent(MOTOR1_CURRENT_SENSOR_PIN, motor1CurrentReadings, motor1ReadIndex, motor1CurrentSum);
      int motor2AvgADC_main = readCurrent(MOTOR2_CURRENT_SENSOR_PIN, motor2CurrentReadings, motor2ReadIndex, motor2CurrentSum);

      Serial.print("MAIN_LOOP - M1_RAW_ADC: "); Serial.print(motor1RawADC_main);
      Serial.print(", M2_RAW_ADC: "); Serial.print(motor2RawADC_main);
      Serial.print(" -- M1_AVG_ADC: "); Serial.print(motor1AvgADC_main);
      Serial.print(", M2_AVG_ADC: "); Serial.print(motor2AvgADC_main);
    }
    // State is always printed
    Serial.print(" -- State: "); Serial.println(getStateName(currentSystemState));
  }
}


// --- Interrupt Service Routines (ISRs) ---
void clampButtonPressed() {
  if ((isBrakePedalPressed() || ignoreBrakePedal) && currentSystemState != CLMP_IN_PROGRESS && currentSystemState != REL_IN_PROGRESS) {
    clampRequested = true;
  }
}

void releaseButtonPressed() {
  if ((isBrakePedalPressed() || ignoreBrakePedal) && currentSystemState != CLMP_IN_PROGRESS && currentSystemState != REL_IN_PROGRESS) {
    releaseRequested = true;
  }
}


// --- Helper Functions ---

bool isBrakePedalPressed() {
  return digitalRead(BRAKE_PEDAL_PIN) == HIGH;
}

String getStateName(SystemState state) {
  switch (state) {
    case RELEASED: return "RELEASED";
    case CLAMPED: return "CLAMPED";
    case CLMP_IN_PROGRESS: return "CLMP_IN_PROGRESS";
    case REL_IN_PROGRESS: return "REL_IN_PROGRESS";
    case FAILURE: return "FAILURE";
    default: return "UNKNOWN";
  }
}

void setSystemState(SystemState newState) {
  currentSystemState = newState;
  // This print is for immediate notification of state changes, always enabled
  Serial.print("System State changed to: ");
  Serial.println(getStateName(currentSystemState));
  updateLedStatus();
}

void updateLedStatus() {
  unsigned long currentTime = millis();
  switch (currentSystemState) {
    case CLAMPED:
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
    case RELEASED:
      digitalWrite(STATUS_LED_PIN, LOW);
      break;
    case CLMP_IN_PROGRESS:
    case REL_IN_PROGRESS:
      if (currentTime - lastLedToggleTime >= LED_BLINK_PROGRESS_MS) {
        lastLedToggleTime = currentTime;
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState);
      }
      break;
    case FAILURE:
      if (currentTime - lastLedToggleTime >= LED_BLINK_FAILURE_MS) {
        lastLedToggleTime = currentTime;
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState);
      }
      break;
  }
}

int readCurrent(int sensorPin, int readings[], int& index, long& sum) {
  sum -= readings[index];
  readings[index] = analogRead(sensorPin);
  sum += readings[index];
  index = (index + 1) % MOVING_AVERAGE_SAMPLES;
  return sum / MOVING_AVERAGE_SAMPLES;
}

void turnOffAllRelays() {
  digitalWrite(MOTOR1_CLAMP_RELAY_PIN, LOW);
  digitalWrite(MOTOR1_RELEASE_RELAY_PIN, LOW);
  digitalWrite(MOTOR2_CLAMP_RELAY_PIN, LOW);
  digitalWrite(MOTOR2_RELEASE_RELAY_PIN, LOW);
  if (ENABLE_SERIAL_DEBUG) { // Debug print
    Serial.println("All relays turned OFF.");
  }
}

void controlMotors(bool clampMode) {
  bool motor1Finished = false;
  bool motor2Finished = false;
  unsigned long startTime = millis();

  turnOffAllRelays();

  if (clampMode) {
    digitalWrite(MOTOR1_CLAMP_RELAY_PIN, HIGH);
    if (ENABLE_SERIAL_DEBUG) { // Debug print
      Serial.println("Activating Motor 1 CLAMP relay...");
    }
    _delay_ms(100); // 100ms delay between motor activations to minimize inrush current
    digitalWrite(MOTOR2_CLAMP_RELAY_PIN, HIGH);
    if (ENABLE_SERIAL_DEBUG) { // Debug print
      Serial.println("Activating Motor 2 CLAMP relay...");
    }
  } else {
    digitalWrite(MOTOR1_RELEASE_RELAY_PIN, HIGH);
    if (ENABLE_SERIAL_DEBUG) { // Debug print
      Serial.println("Activating Motor 1 RELEASE relay...");
    }
    _delay_ms(100); // 100ms delay between motor activations to minimize inrush current
    digitalWrite(MOTOR2_RELEASE_RELAY_PIN, HIGH);
    if (ENABLE_SERIAL_DEBUG) { // Debug print
      Serial.println("Activating Motor 2 RELEASE relay...");
    }
  }

  while (!motor1Finished || !motor2Finished) {
    wdt_reset();

    if (millis() - startTime >= OPERATION_TIMEOUT_MS) {
      Serial.println("Operation Timeout!"); // Always print timeout
      turnOffAllRelays();
      setSystemState(FAILURE); // This will also print the state change
      return;
    }

    updateLedStatus();

    int motor1AvgCurrent = CURRENT_QUIESCENT_ADC;
    if (!motor1Finished) {
      motor1AvgCurrent = readCurrent(MOTOR1_CURRENT_SENSOR_PIN, motor1CurrentReadings, motor1ReadIndex, motor1CurrentSum);
      if (abs(motor1AvgCurrent - CURRENT_QUIESCENT_ADC) > CURRENT_THRESHOLD_ADC_DIFF) {
        if (ENABLE_SERIAL_DEBUG) { // Debug print
          Serial.print("CONTROL_LOOP - Motor 1 current spike detected (ADC: ");
          Serial.print(motor1AvgCurrent);
          Serial.println(")! Turning off Motor 1.");
        }
        _delay_ms(100);
        if (clampMode) {
          digitalWrite(MOTOR1_CLAMP_RELAY_PIN, LOW);
        } else {
          digitalWrite(MOTOR1_RELEASE_RELAY_PIN, LOW);
        }
        motor1Finished = true;
      }
    }

    int motor2AvgCurrent = CURRENT_QUIESCENT_ADC;
    if (!motor2Finished) {
      motor2AvgCurrent = readCurrent(MOTOR2_CURRENT_SENSOR_PIN, motor2CurrentReadings, motor2ReadIndex, motor2CurrentSum);
      if (abs(motor2AvgCurrent - CURRENT_QUIESCENT_ADC) > CURRENT_THRESHOLD_ADC_DIFF) {
        if (ENABLE_SERIAL_DEBUG) { // Debug print
          Serial.print("CONTROL_LOOP - Motor 2 current spike detected (ADC: ");
          Serial.print(motor2AvgCurrent);
          Serial.println(")! Turning off Motor 2.");
        }
        _delay_ms(100);
        if (clampMode) {
          digitalWrite(MOTOR2_CLAMP_RELAY_PIN, LOW);
        } else {
          digitalWrite(MOTOR2_RELEASE_RELAY_PIN, LOW);
        }
        motor2Finished = true;
      }
    }

    // Print current values during operation (more frequent) if debugging is enabled
    unsigned long currentControlTime = millis();
    if (ENABLE_SERIAL_DEBUG && (currentControlTime - lastSerialReportTimeControl >= SERIAL_REPORT_INTERVAL_CONTROL_MS)) {
        lastSerialReportTimeControl = currentControlTime;
        Serial.print("CONTROL_LOOP - M1_AVG: "); Serial.print(motor1AvgCurrent);
        Serial.print(", M2_AVG: "); Serial.println(motor2AvgCurrent);
    }

    _delay_ms(10);
  }

  // Both motors finished successfully
  turnOffAllRelays();

  // Always print successful operation status
  if (clampMode) {
    setSystemState(CLAMPED); // This will also print the state change
    EEPROM.update(EEPROM_STATE_ADDRESS, CLAMPED);
    Serial.println("Clamping operation SUCCESSFUL. System CLAMPED.");
  } else {
    setSystemState(RELEASED); // This will also print the state change
    EEPROM.update(EEPROM_STATE_ADDRESS, RELEASED);
    Serial.println("Releasing operation SUCCESSFUL. System RELEASED.");
  }
}
