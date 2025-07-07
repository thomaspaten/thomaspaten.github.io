/*
VERY IMPORTANT
For <dimmable_light_linearized.h> to work, Arduino AVR board must be version 1.8.2 or lower 
(Menu - Tool - Board - Boards manager)
*/

/*
Summary of Conditions for Dimmers and Fans Adjustments

                  TEMPERATURE         DIMMER        FANS                          TARGET
Top
Very Low Temp     (< 36.0°C)	        High (80%)	  F4HG: High (100%)             -
Low Temp          (36.0°C - 37.35°C)	Low (40%)	    F4HG: Medium (75%)            -
Medium Temp       (37.35°C - 37.45°C)	Off (0%)	    F4HG: Low (50%)               37.2–37.5°C
High Temp         (> 37.45°C)	        Off (0%)	    F4HG: High (100%)             -

Very Low Humidity (< 55.0%)	          -	            F7HD: High (100%)             - 
Low Humidity      (55.0% - 65.0%)	    -	            F7HD: Medium (75%)            -
Medium Humidity   (65.0% - 74.0%)	    -	            F7HD: Very Low (25%)          65–75%
High Humidity     (> 74.0%)	          -	            F7HD: Off (0%)

Bot
Very Low Temp     (< 36.0°C)	        High (80%)	  F1BG_F2BD_F3BC: High (100%)   -
Low Temp          (36.0°C - 37.45°C)	Low (40%)	F1BG_F2BD_F3BC: Medium (75%)  -
Medium Temp       (37.45°C - 37.55°C)	Off (0%)	    F1BG_F2BD_F3BC: Low (50%)     37.5°C
High Temp         (> 37.55°C)	        Off (0%)	    F1BG_F2BD_F3BC: High (100%)   -

Very Low Humidity (< 40.0%)	          -	            F6BC: High (100%)             -
Low Humidity      (40.0% - 50.0%)	    -	            F6BC: Medium (75%)            -
Medium Humidity   (50.0% - 55.0%)	    -	            F6BC: Very Low (25%)          50–55%
High Humidity     (> 55.0%)	          -	            F6BC: Off (0%)                -

Ext
High Temp         (> 36.8°C)	        -	            F5EXT: High (100%)            -
Otherwise	                            -	            F5EXT: Very Low (25%)         -
*/

#include <Wire.h>                       // Library for I2C communication (sensors and multiplexer)
#include <Adafruit_AHTX0.h>             // Library for AHT20 temperature/humidity sensor
#include <dimmable_light_linearized.h>  // Library for controlling dimmable lights
#include <avr/wdt.h>                    // Library for watchdog timer (system reset on failure)

// ====================
// Constants
// ====================

// Printing cycles
constexpr int PRINT_CYCLE_INTERVAL = 30;  // Interval (in cycles) for printing debug info
constexpr int N_READINGS = 100;            // Number of readings for sensor temperature averaging

// ====================
// Pin Definitions
// ====================

// PWM Fan Pins (Default Arduino Mega frequency)
constexpr int PWM_F1BG_F2BD_F3BC = 2;  // Shared PWM pin for fans F1BG, F2BD, F3BC (Timer 3)
constexpr int PWM_F4HG = 3;            // PWM pin for fan F4HG (Timer 3)
constexpr int PWM_F5EXT = 5;           // PWM pin for fan F5EXT (Timer 3)
constexpr int PWM_F6BC = 45;           // PWM pin for fan F6BC (Timer 5)
constexpr int PWM_F7HD = 44;           // PWM pin for fan F7HD (Timer 5)

// TACH Fan Pins (for reading fan speed)
constexpr int TACH_F1BG = 42;   // TACH pin for fan F1BG (reads RPM)
constexpr int TACH_F2BD = 38;   // TACH pin for fan F2BD (reads RPM)
constexpr int TACH_F3BC = 40;   // TACH pin for fan F3BC (reads RPM)
constexpr int TACH_F4HG = 51;   // TACH pin for fan F4HG (reads RPM)
constexpr int TACH_F5EXT = 43;  // TACH pin for fan F5EXT (reads RPM)
constexpr int TACH_F6BC = 41;   // TACH pin for fan F6BC (reads RPM)
constexpr int TACH_F7HD = 47;   // TACH pin for fan F7HD (reads RPM)

// I2C Multiplexer Address
constexpr int TCA9548A_ADDRESS = 0x70;  // Address of TCA9548A I2C multiplexer

// Dimmable Light Pins
constexpr int dimmerZeroCrossPin = 18;  // Pin for zero-crossing detection (dimmer control)
constexpr int topDimmerOutputPin = 6;   // Pin for top dimmer output (brightness control)
constexpr int botDimmerOutputPin = 7;   // Pin for bottom dimmer output (brightness control)

// Motor driver pins (Linear Actuator)
constexpr int R_EN = 48;   // Right enable pin for motor driver
constexpr int L_EN = 49;   // Left enable pin for motor driver
constexpr int R_PWM = 10;  // Right PWM pin for motor driver
constexpr int L_PWM = 9;   // Left PWM pin for motor driver

// I2C Bus Reset Pins (assuming SDA = 20, SCL = 21 on Arduino Mega)
constexpr int SDA_PIN = 20;  // I2C SDA pin
constexpr int SCL_PIN = 21;  // I2C SCL pin

// ====================
// Constants
// ====================

// Number of Sensors
constexpr int NUM_SENSORS = 4;  // Total number of temp/humidity sensors

// Sensor Names (used to identify sensors in code and output)
enum SensorName { EXT, TOP, BOT_D, BOT_G };  // Enum for sensor naming

// Convert SensorName enum to readable string
const char* getSensorName(SensorName sensor) {
  switch (sensor) {
    case EXT: return "EXT";
    case TOP: return "TOP";
    case BOT_D: return "BOT_D";
    case BOT_G: return "BOT_G";
    default: return "UNKNOWN";
  }
}

// Temperature Thresholds
constexpr float VERY_LOW_TEMP_TOP = 36;    // Very low temp threshold for TOP sensor (°C)
constexpr float LOW_TEMP_TOP = 37.50;      // Low temp threshold for TOP sensor (°C) was 37.38
constexpr float HIGH_TEMP_TOP = 37.60;     // High temp threshold for TOP sensor (°C) was 37.48

constexpr float VERY_LOW_TEMP_BOT = 36;    // Very low temp threshold for BOT sensors (°C)
constexpr float LOW_TEMP_BOT = 37.54;      // Low temp threshold for BOT sensors (°C)
constexpr float HIGH_TEMP_BOT = 37.68;     // High temp threshold for BOT sensors (°C)

constexpr float VERY_LOW_TEMP_EXT = 18.0;  // Very low temp threshold for EXT sensor (°C)
constexpr float LOW_TEMP_EXT = 22.0;       // Low temp threshold for EXT sensor (°C)
constexpr float HIGH_TEMP_EXT = 36.8;      // High temp threshold for EXT sensor (°C)

// Humidity Thresholds
constexpr float VERY_LOW_HUMIDITY_TOP = 55.0;  // Very low humidity threshold for TOP (%)
constexpr float LOW_HUMIDITY_TOP = 62.0;       // Low humidity threshold for TOP (%) was 65
constexpr float MEDIUM_HUMIDITY_TOP = 68.0;    // Medium humidity threshold for TOP (%) was 74
constexpr float HIGH_HUMIDITY_TOP = 68.0;      // High humidity threshold for TOP (%) was 74

constexpr float VERY_LOW_HUMIDITY_BOT = 40.0;  // Very low humidity threshold for BOT (%)
constexpr float LOW_HUMIDITY_BOT = 45.0;       // Low humidity threshold for BOT (%) était 50
constexpr float MEDIUM_HUMIDITY_BOT = 48.0;    // Medium humidity threshold for BOT (%) était 55
constexpr float HIGH_HUMIDITY_BOT = 52.0;      // High humidity threshold for BOT (%) était 55

// Fan Speed Thresholds (in percentage)
constexpr int FAN_OFF = 0;        // Fan off (0% speed)
constexpr int FAN_VERY_LOW = 25;  // Fan speed at 25%
constexpr int FAN_LOW = 50;       // Fan speed at 50%
constexpr int FAN_MEDIUM = 75;    // Fan speed at 75%
constexpr int FAN_HIGH = 100;     // Fan speed at 100%

// Dimmer Brightness Thresholds (in percentage)
constexpr int DIMMER_OFF = 0;      // Dimmer off (0% brightness)
constexpr int DIMMER_LOW = 40;     // Dimmer brightness at 40%
constexpr int DIMMER_MEDIUM = 60;  // Dimmer brightness at 60%
constexpr int DIMMER_HIGH = 80;    // Dimmer brightness at 80%

// PWM and Timing Constants
constexpr int PWM_MAX_VALUE = 255;  // Max value for 8-bit PWM (100%)

// Sensor Timing
constexpr long readInterval = 3000;        // Interval for reading all sensors (ms)
constexpr long sensorDelayInterval = 500;  // Delay between individual sensor reads (ms)

// Error Handling Constants
constexpr int MAX_READ_ATTEMPTS = 3;    // Max attempts to read a sensor
constexpr int DEFAULT_TEMP = 25.0;      // Default temp if EXT sensor fails (°C)
constexpr int DEFAULT_HUMIDITY = 50.0;  // Default humidity if sensor fails (%)
constexpr int MAX_FAILURES_BEFORE_DISABLE = 10;  // Failures before disabling sensor

// Watchdog Timer Timeout
constexpr int WATCHDOG_TIMEOUT = 8000;  // 8s watchdog timeout (ms)

// Actuator Timing Constants
constexpr unsigned long RETRACT_DURATION = 60000;       // 60s initial retraction (ms)
constexpr unsigned long INITIAL_STOP_DURATION = 10000;  // 10s initial stop (ms)
constexpr unsigned long EXTEND_TIME = 3000;             // 3s extend time (ms)
constexpr unsigned long RETRACT_TIME = 3000;            // 3s retract time (ms)
constexpr unsigned long STOP_TIME = 780000;             // 13 min stop time (ms)
constexpr int REPETITIONS = 11;                         // Number of actuator cycles

// Actuator State Machine
enum State {
  RETRACTING_INITIAL,   // Initial retraction phase
  STOPPING_INITIAL,     // Initial stop phase
  EXTENDING_SEQUENCE,   // Extending sequence phase
  RETRACTING_SEQUENCE,  // Retracting sequence phase
  STOPPING              // Stop phase
};

// Pulse Measurement Constants
constexpr unsigned long MIN_PULSE_DURATION = 1000;    // Min valid pulse duration (µs)
constexpr unsigned long MAX_PULSE_DURATION = 1000000; // Max valid pulse duration (µs)

// Valid Range for Temperature and Humidity
constexpr float MIN_VALID_TEMP = -40.0;    // Min valid temperature (°C)
constexpr float MAX_VALID_TEMP = 85.0;     // Max valid temperature (°C)
constexpr float MIN_VALID_HUMIDITY = 0.0;  // Min valid humidity (%)
constexpr float MAX_VALID_HUMIDITY = 100.0;// Max valid humidity (%)

// I2C Retry Constants
constexpr int MAX_I2C_RETRIES = 3;      // Max retries for I2C communication
constexpr int RETRY_DELAY = 50;         // Delay between I2C retries (ms)
constexpr int BUS_RESET_DELAY = 100;    // Delay after bus reset (ms)
constexpr int BUS_RECOVERY_INTERVAL = 60000;  // Interval to reattempt recovery (ms)

// Sensor Initialization Timeout Constants
constexpr unsigned long SENSOR_INIT_TIMEOUT = 5000;  // 5s timeout per sensor (ms)
constexpr unsigned long TOTAL_INIT_TIMEOUT = 30000;  // 30s total init timeout (ms)

// ====================
// Global Variables
// ====================

// Fan Control Variables
unsigned long previousSensorMillis = 0;  // Last time sensors were read
unsigned long sensorDelayMillis = 0;     // Delay tracker between sensor reads
int sensorIndex = 0;                     // Current sensor being read
bool readingCycleComplete = false;       // Flag for completed sensor cycle

// Dimmable Light Objects
DimmableLightLinearized topLight(topDimmerOutputPin);  // Top dimmer control
DimmableLightLinearized botLight(botDimmerOutputPin);  // Bottom dimmer control

// AHT20 Sensor Object
Adafruit_AHTX0 aht20;  // Single AHT20 sensor instance

// Bot Sensor Data Structure
struct BotSensorData {
  float temperatureD = DEFAULT_TEMP;   // BOT_D temperature (°C)
  float temperatureG = DEFAULT_TEMP;   // BOT_G temperature (°C)
  float humidityD = DEFAULT_HUMIDITY;  // BOT_D humidity (%)
  float humidityG = DEFAULT_HUMIDITY;  // BOT_G humidity (%)
  bool updatedD = false;               // BOT_D update flag
  bool updatedG = false;               // BOT_G update flag
};

BotSensorData botSensorData;  // Bot sensor data instance

// Sensor Health Tracking Structure
struct SensorHealth {
  int successfulReads = 0;  // Count of successful reads
  int failedReads = 0;      // Count of failed reads
};

SensorHealth sensorHealth[NUM_SENSORS];  // Health data for each sensor
bool disabledSensors[NUM_SENSORS] = {false};  // Disabled sensor flags
unsigned long lastSensorCheckMillis = 0;      // Last sensor health check time
constexpr long SENSOR_CHECK_INTERVAL = 60000; // Check disabled sensors every 60s

// Sensor Initialization Tracking
bool sensorInitialized[NUM_SENSORS] = {false};  // Initialization flags

// Cycle Counter
int cycleCounter = 0;  // Tracks completed sensor reading cycles

// Actuator Variables
State currentState = RETRACTING_INITIAL;   // Current actuator state
State previousState = RETRACTING_INITIAL;  // Previous actuator state
unsigned long actuatorPreviousMillis = 0;  // Last actuator state update
int sequenceCount = 0;                     // Actuator sequence counter

// Temperature Reading Arrays
float topTemperatures[N_READINGS] = {0};   // TOP temp readings
float botDTemperatures[N_READINGS] = {0};  // BOT_D temp readings
float botGTemperatures[N_READINGS] = {0};  // BOT_G temp readings

// Humidity Reading Arrays
float topHumidities[N_READINGS] = {0};     // TOP humidity readings
float botDHumidities[N_READINGS] = {0};    // BOT_D humidity readings
float botGHumidities[N_READINGS] = {0};    // BOT_G humidity readings

int readingIndex = 0;                      // Current reading index

// Pulse Measurement Structure
struct PulseData {
  unsigned long startTime = 0;  // Pulse start time (µs)
  unsigned long duration = 0;   // Pulse duration (µs)
  bool pulseDetected = false;   // Pulse detection flag
};

PulseData pulseData[7];  // Pulse data for each fan

// I2C Retry Variables
unsigned long lastI2CRetryMillis = 0;  // Last I2C retry time
unsigned long lastBusResetMillis = 0;  // Last bus reset time
bool busResetInProgress = false;       // Flag for bus reset state

// Next Sequence Tracking
State nextSequenceState = RETRACTING_INITIAL;  // Predicted next significant state
int nextSequenceCount = 0;                     // Next sequence repetition number

// External Temperature Tracking
float extTemperature = DEFAULT_TEMP;  // Store EXT sensor temperature

// ====================
// Non-Blocking Pulse Measurement
// ====================

// Measure fan pulses without blocking
void measurePulseNonBlocking(int tachPin, int fanIndex) {
  int pulseState = digitalRead(tachPin);  // Read TACH pin state
  if (pulseState == HIGH && !pulseData[fanIndex].pulseDetected) {
    pulseData[fanIndex].startTime = micros();  // Start pulse timing
    pulseData[fanIndex].pulseDetected = true;  // Mark pulse start
  } else if (pulseState == LOW && pulseData[fanIndex].pulseDetected) {
    pulseData[fanIndex].duration = micros() - pulseData[fanIndex].startTime;  // Calculate duration
    pulseData[fanIndex].pulseDetected = false;  // Reset for next pulse
  }
}

// Calculate RPM with validation
unsigned long calculateRPMNonBlocking(int fanIndex) {
  unsigned long duration = pulseData[fanIndex].duration;
  if (duration >= MIN_PULSE_DURATION && duration <= MAX_PULSE_DURATION) {
    unsigned long rpm = 60000000 / (duration * 4);  // RPM calculation (2 pulses per revolution)
    pulseData[fanIndex].duration = 0;  // Reset duration
    return rpm;
  }
  pulseData[fanIndex].duration = 0;  // Reset invalid duration
  return 0;  // Invalid pulse, return 0 RPM
}

// ====================
// Helper Functions
// ====================

// Convert percentage to PWM value
uint8_t percentageToPWM(uint8_t percentage) {
  return map(percentage, 0, 100, 0, PWM_MAX_VALUE);  // Map 0-100% to 0-255
}

// Check if it's time to print debug info
bool shouldPrint() {
  return cycleCounter % PRINT_CYCLE_INTERVAL == 0;
}

// Reset I2C bus to recover from lockup
void resetI2CBus() {
  if (shouldPrint()) {
    Serial.println("Attempting I2C bus reset...");
  }

  // Set SDA and SCL as outputs
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);

  // Clock out any stuck devices (up to 9 pulses)
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }

  // Generate a stop condition
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);

  // Return pins to I2C control
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);

  Wire.begin();  // Reinitialize I2C
  delay(BUS_RESET_DELAY);  // Wait for bus stabilization

  if (shouldPrint()) {
    Serial.println("I2C bus reset complete.");
  }
}

// Select I2C multiplexer channel with retries and bus reset
bool selectChannel(uint8_t channel) {
  if (channel > 7) return false;  // Validate channel (0-7)

  static int retryCount = 0;      // Retry counter (static to reset on success/failure)
  static int i2cState = 0;        // 0: idle, 1: transmitting, 2: retrying, 3: resetting
  static int result = -1;         // I2C transmission result
  static uint8_t lastChannel = 255;  // Track last channel to reset state on change

  unsigned long currentMillis = millis();

  // Reset state if channel changes
  if (channel != lastChannel) {
    i2cState = 0;
    retryCount = 0;
    result = -1;
    busResetInProgress = false;
    lastChannel = channel;
  }

  // Handle I2C state machine
  switch (i2cState) {
    case 0:  // Idle state, start transmission
      Wire.beginTransmission(TCA9548A_ADDRESS);  // Start I2C communication
      Wire.write(1 << channel);                  // Select channel
      result = Wire.endTransmission();           // Send and check result
      if (result == 0) {
        i2cState = 0;  // Success, reset state
        retryCount = 0;
        busResetInProgress = false;
        return true;
      } else {
        retryCount++;
        if (shouldPrint()) {
          Serial.print("I2C retry ");
          Serial.print(retryCount);
          Serial.print(" failed on channel ");
          Serial.println(channel);
        }
        i2cState = 2;  // Move to retry state
        lastI2CRetryMillis = currentMillis;  // Record retry start time
        return false;  // In progress
      }
      break;

    case 2:  // Retry state
      if (retryCount >= MAX_I2C_RETRIES) {
        if (shouldPrint()) {
          Serial.print("Max I2C retries exceeded on channel ");
          Serial.println(channel);
        }
        i2cState = 3;  // Move to reset state
        lastI2CRetryMillis = currentMillis;
        busResetInProgress = true;
        resetI2CBus();  // Attempt bus reset
        return false;  // Proceed to reset
      }
      if (currentMillis - lastI2CRetryMillis >= RETRY_DELAY) {
        Wire.beginTransmission(TCA9548A_ADDRESS);  // Retry transmission
        Wire.write(1 << channel);
        result = Wire.endTransmission();
        if (result == 0) {
          i2cState = 0;  // Success, reset state
          retryCount = 0;
          busResetInProgress = false;
          return true;
        } else {
          retryCount++;
          if (shouldPrint()) {
            Serial.print("I2C retry ");
            Serial.print(retryCount);
            Serial.print(" failed on channel ");
            Serial.println(channel);
          }
          lastI2CRetryMillis = currentMillis;  // Update retry time
          return false;  // In progress
        }
      }
      return false;  // Waiting for retry delay

    case 3:  // Bus reset state
      if (currentMillis - lastI2CRetryMillis >= BUS_RESET_DELAY) {
        Wire.beginTransmission(TCA9548A_ADDRESS);  // Retry after reset
        Wire.write(1 << channel);
        result = Wire.endTransmission();
        if (result == 0) {
          i2cState = 0;  // Success, reset state
          retryCount = 0;
          busResetInProgress = false;
          if (shouldPrint()) {
            Serial.print("I2C communication restored on channel ");
            Serial.println(channel);
          }
          return true;
        } else {
          if (shouldPrint()) {
            Serial.print("Error: I2C communication failed on channel ");
            Serial.print(channel);
            Serial.println(" after bus reset - using fallback values.");
          }
          i2cState = 0;  // Reset state, use fallback
          retryCount = 0;
          busResetInProgress = false;
          lastBusResetMillis = currentMillis;  // Track last reset
          return false;  // Definitive failure, use fallback
        }
      }
      return false;  // Waiting for reset delay
  }
  return false;  // Default (in progress)
}

// Initialize sensor on a specific channel with timeout
bool initializeSensor(int channel, unsigned long startTime, bool& timedOut) {
  int sensorIdx = channel - 2;  // Map channel (2-5) to sensor index (0-3)
  if (sensorIdx < 0 || sensorIdx >= NUM_SENSORS) return false;

  if (sensorInitialized[sensorIdx]) return true;  // Skip if already initialized

  unsigned long currentMillis = millis();
  if (currentMillis - startTime >= SENSOR_INIT_TIMEOUT) {
    if (shouldPrint()) {
      Serial.print("Error: Timeout initializing sensor on channel ");
      Serial.println(channel);
    }
    timedOut = true;
    return false;  // Per-sensor timeout
  }

  if (selectChannel(channel)) {  // Keep calling until success
    if (aht20.begin()) {  // Initialize AHT20 sensor
      sensorInitialized[sensorIdx] = true;  // Mark as initialized
      if (shouldPrint()) {
        Serial.print("Sensor initialized on channel ");
        Serial.println(channel);
      }
      return true;
    } else {
      if (shouldPrint()) {
        Serial.print("Error: Failed to initialize AHT20 sensor on channel ");
        Serial.println(channel);
      }
      return false;  // Failed but not timed out yet
    }
  }
  return false;  // Still in progress
}

// Check and potentially disable a sensor due to failures
void checkAndDisableSensor(int sensorIndex) {
  if (sensorHealth[sensorIndex].failedReads >= MAX_FAILURES_BEFORE_DISABLE) {
    disabledSensors[sensorIndex] = true;  // Disable sensor
    if (shouldPrint()) {
      Serial.print("Sensor ");
      Serial.print(getSensorName(static_cast<SensorName>(sensorIndex)));
      Serial.println(" disabled due to repeated failures.");
    }
  }
}

// Periodically check and re-enable disabled sensors
void checkDisabledSensors() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorCheckMillis >= SENSOR_CHECK_INTERVAL) {
    lastSensorCheckMillis = currentMillis;  // Update last check time
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (disabledSensors[i]) {  // Check each disabled sensor
        if (selectChannel(i + 2) && aht20.begin()) {  // Attempt to re-initialize
          disabledSensors[i] = false;  // Re-enable sensor
          sensorHealth[i].failedReads = 0;  // Reset failure count
          sensorInitialized[i] = true;  // Mark as initialized
          if (shouldPrint()) {
            Serial.print("Sensor ");
            Serial.print(getSensorName(static_cast<SensorName>(i)));
            Serial.println(" re-enabled after recovery.");
          }
        }
      }
    }
  }
}

// Get fallback temperature for a failed sensor
float getFallbackTemperature(int sensorIndex) {
  switch (sensorIndex) {
    case EXT: return DEFAULT_TEMP;              // Default for EXT
    case TOP: return botSensorData.temperatureD - 0.3;  // Fallback for TOP
    case BOT_D: return botSensorData.temperatureG;      // Fallback for BOT_D
    case BOT_G: return botSensorData.temperatureD;      // Fallback for BOT_G
    default: return DEFAULT_TEMP;               // Default for unknown
  }
}

// Get fallback humidity for a failed sensor
float getFallbackHumidity(int sensorIndex) {
  switch (sensorIndex) {
    case EXT: return DEFAULT_HUMIDITY;          // Default for EXT
    case TOP: return botSensorData.humidityD + 15.0;  // Fallback for TOP
    case BOT_D: return botSensorData.humidityG;       // Fallback for BOT_D
    case BOT_G: return botSensorData.humidityD;       // Fallback for BOT_G
    default: return DEFAULT_HUMIDITY;           // Default for unknown
  }
}

// Read sensor data with enhanced error handling
bool readSensorData(int channel, int sensorIndex, sensors_event_t& humidity, sensors_event_t& temp) {
  static bool selectingChannel = false;  // Track channel selection state
  static unsigned long lastAttemptMillis = 0;  // Last attempt time
  const int READ_ATTEMPT_DELAY = 50;  // Delay between attempts (ms)

  unsigned long currentMillis = millis();

  // Non-blocking channel selection
  if (!selectingChannel) {
    selectingChannel = true;  // Start selection process
    lastAttemptMillis = currentMillis;
  }

  if (!selectChannel(channel)) {  // Keep calling until success or failure
    if (currentMillis - lastAttemptMillis >= (RETRY_DELAY * MAX_I2C_RETRIES + BUS_RESET_DELAY)) {
      if (shouldPrint()) {
        Serial.print("Error: Failed to select channel ");
        Serial.println(channel);
      }
      selectingChannel = false;  // Reset state
      return false;  // Timeout or failure after reset
    }
    return false;  // Still in progress
  }

  if (disabledSensors[sensorIndex]) {
    selectingChannel = false;  // Reset state
    return false;  // Skip disabled sensors
  }

  unsigned long retryStartTime = millis();
  for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; attempt++) {
    if (aht20.getEvent(&humidity, &temp)) {  // Attempt to read sensor
      sensorHealth[sensorIndex].successfulReads++;  // Increment success count
      selectingChannel = false;  // Reset state
      return true;
    }

    // Log detailed error information
    if (shouldPrint()) {
      Serial.print("Error reading sensor ");
      Serial.print(getSensorName(static_cast<SensorName>(sensorIndex)));
      Serial.print(" on attempt ");
      Serial.print(attempt + 1);
      Serial.print(". I2C error code: ");
      Serial.println(Wire.endTransmission());  // Log I2C error code
    }

    // Non-blocking delay
    while (millis() - retryStartTime < 100) {
      wdt_reset();  // Prevent watchdog timeout
    }
    retryStartTime = millis();
  }

  sensorHealth[sensorIndex].failedReads++;  // Increment failure count
  checkAndDisableSensor(sensorIndex);       // Check if sensor should be disabled
  selectingChannel = false;  // Reset state
  return false;
}

// Print sensor health status
void printSensorHealth() {
  if (shouldPrint()) {
    Serial.println("=========================================");
    Serial.println("Sensor Health Status:");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(getSensorName(static_cast<SensorName>(i)));
      Serial.print(" - Successful Reads: ");
      Serial.print(sensorHealth[i].successfulReads);
      Serial.print(", Failed Reads: ");
      Serial.println(sensorHealth[i].failedReads);
    }
  }
}

// Log actuator state changes
void logStateChange() {
  if (currentState != previousState) {
    if (shouldPrint()) {
      switch (currentState) {
        case RETRACTING_INITIAL: Serial.println("Retracting to starting point..."); break;
        case STOPPING_INITIAL: Serial.println("Initial retraction complete! 10s stop..."); break;
        case EXTENDING_SEQUENCE: Serial.println("Starting extension sequence..."); break;
        case RETRACTING_SEQUENCE: Serial.println("Starting retraction sequence..."); break;
        case STOPPING: Serial.println("Linear Actuator stopping..."); break;
      }
    }
    previousState = currentState;  // Update previous state
  }
}

// Control motor with pin state validation
void controlMotor(bool enable, int dir) {
  // Ensure pins are in a valid state before enabling/disabling
  if (enable) {
    digitalWrite(R_PWM, dir == 1 ? HIGH : LOW);  // Set right direction
    digitalWrite(L_PWM, dir == -1 ? HIGH : LOW); // Set left direction
    digitalWrite(R_EN, HIGH);                    // Enable right driver
    digitalWrite(L_EN, HIGH);                    // Enable left driver
  } else {
    digitalWrite(R_EN, LOW);                     // Disable right driver
    digitalWrite(L_EN, LOW);                     // Disable left driver
    digitalWrite(R_PWM, LOW);                    // Clear right PWM
    digitalWrite(L_PWM, LOW);                    // Clear left PWM
  }
}

// Calculate average of last N readings
float calculateAverage(float readings[], int n) {
  float sum = 0;
  for (int i = 0; i < n; i++) {
    sum += readings[i];
  }
  return sum / n;
}

// Print averages of last N temperature readings
void printAverages() {
  if (shouldPrint()) {
    float avgTopTemp = calculateAverage(topTemperatures, N_READINGS);
    float avgBotDTemp = calculateAverage(botDTemperatures, N_READINGS);
    float avgBotGTemp = calculateAverage(botGTemperatures, N_READINGS);

    Serial.print("TOP Average 100 Temperature:   ");
    Serial.print(avgTopTemp);
    Serial.println(" °C");
    Serial.print("BOT_D Average 100 Temperature: ");
    Serial.print(avgBotDTemp);
    Serial.println(" °C");
    Serial.print("BOT_G Average 100 Temperature: ");
    Serial.print(avgBotGTemp);
    Serial.println(" °C");
    Serial.println();
  }
}

// Print averages of last N humidity readings
void printHumidityAverages() {
  if (shouldPrint()) {
    float avgTopHumidity = calculateAverage(topHumidities, N_READINGS);
    float avgBotDHumidity = calculateAverage(botDHumidities, N_READINGS);
    float avgBotGHumidity = calculateAverage(botGHumidities, N_READINGS);

    Serial.print("TOP Average 100 Humidity:   ");
    Serial.print(avgTopHumidity);
    Serial.println(" %");
    Serial.print("BOT_D Average 100 Humidity: ");
    Serial.print(avgBotDHumidity);
    Serial.println(" %");
    Serial.print("BOT_G Average 100 Humidity: ");
    Serial.print(avgBotGHumidity);
    Serial.println(" %");
    Serial.println();
  }
}

// Validate sensor readings
bool isValidReading(float value, float minVal, float maxVal) {
  return !isnan(value) && value >= minVal && value <= maxVal;
}

// Predict the next significant actuator state
void predictNextSequence() {
  State tempState = currentState;
  int tempSequenceCount = sequenceCount;

  while (true) {
    switch (tempState) {
      case RETRACTING_INITIAL:
        tempState = STOPPING_INITIAL;
        break;
      case STOPPING_INITIAL:
        tempState = EXTENDING_SEQUENCE;
        tempSequenceCount = 0;
        nextSequenceState = tempState;
        nextSequenceCount = tempSequenceCount + 1;
        return;
      case EXTENDING_SEQUENCE:
        if (tempSequenceCount < REPETITIONS - 1) {
          tempSequenceCount++;
          nextSequenceState = tempState;
          nextSequenceCount = tempSequenceCount + 1;
          return;
        } else {
          tempState = RETRACTING_SEQUENCE;
          tempSequenceCount = 0;
          nextSequenceState = tempState;
          nextSequenceCount = tempSequenceCount + 1;
          return;
        }
      case RETRACTING_SEQUENCE:
        if (tempSequenceCount < REPETITIONS - 1) {
          tempSequenceCount++;
          nextSequenceState = tempState;
          nextSequenceCount = tempSequenceCount + 1;
          return;
        } else {
          tempState = RETRACTING_INITIAL;
          tempSequenceCount = 0;
          nextSequenceState = tempState;
          nextSequenceCount = 1;
          return;
        }
      case STOPPING:
        if (currentState == EXTENDING_SEQUENCE) {
          tempState = EXTENDING_SEQUENCE;
        } else if (currentState == RETRACTING_SEQUENCE) {
          tempState = RETRACTING_SEQUENCE;
        }
        break;
    }
  }
}

// Print the next sequence message
void printNextSequence() {
  if (shouldPrint()) {
    predictNextSequence();
    Serial.print("Next Linear actuator sequence: ");
    Serial.print(nextSequenceCount);
    Serial.print(" of ");
    Serial.print(REPETITIONS);
    Serial.print(" ");
    switch (nextSequenceState) {
      case RETRACTING_INITIAL: Serial.println("RETRACTING_INITIAL"); break;
      case EXTENDING_SEQUENCE: Serial.println("EXTENDING_SEQUENCE"); break;
      case RETRACTING_SEQUENCE: Serial.println("RETRACTING_SEQUENCE"); break;
      default: Serial.println("UNKNOWN"); break;  // Should not occur
    }
    Serial.println("=========================================");
    Serial.println();
  }
}

// ====================
// Setup Function
// ====================

void setup() {
  Serial.begin(115200);  // Start Serial communication
  if (shouldPrint()) {
    Serial.println("Initializing Integrated Sketch...");
  }

  wdt_enable(WDTO_8S);  // Enable watchdog with 8s timeout

  // Configure fan PWM pins
  pinMode(PWM_F1BG_F2BD_F3BC, OUTPUT);
  pinMode(PWM_F4HG, OUTPUT);
  pinMode(PWM_F5EXT, OUTPUT);
  pinMode(PWM_F6BC, OUTPUT);
  pinMode(PWM_F7HD, OUTPUT);

  // Configure fan TACH pins
  pinMode(TACH_F1BG, INPUT);
  pinMode(TACH_F2BD, INPUT);
  pinMode(TACH_F3BC, INPUT);
  pinMode(TACH_F4HG, INPUT);
  pinMode(TACH_F5EXT, INPUT);
  pinMode(TACH_F6BC, INPUT);
  pinMode(TACH_F7HD, INPUT);

  // Configure motor driver pins
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  // Initialize motor driver to off
  controlMotor(false, 0);

  // Initialize fans to 0% speed
  analogWrite(PWM_F1BG_F2BD_F3BC, 0);
  analogWrite(PWM_F4HG, 0);
  analogWrite(PWM_F5EXT, 0);
  analogWrite(PWM_F6BC, 0);
  analogWrite(PWM_F7HD, 0);
  if (shouldPrint()) {
    Serial.println("Fans initialized to 0% speed.");
  }

  // Initialize dimmable lights
  DimmableLightLinearized::setSyncPin(dimmerZeroCrossPin);
  DimmableLightLinearized::begin();
  if (shouldPrint()) {
    Serial.println("Dimmable light control initialized!");
  }

  // Initialize I2C and sensors with enhanced timeout handling
  Wire.begin();
  unsigned long initStartTime = millis();
  bool allSensorsInitialized = false;

  // Track individual sensor initialization attempts
  unsigned long sensorStartTimes[NUM_SENSORS] = {initStartTime, initStartTime, initStartTime, initStartTime};
  bool sensorTimedOut[NUM_SENSORS] = {false};

  while (!allSensorsInitialized && (millis() - initStartTime < TOTAL_INIT_TIMEOUT)) {
    allSensorsInitialized = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (!sensorInitialized[i] && !sensorTimedOut[i]) {
        if (initializeSensor(i + 2, sensorStartTimes[i], sensorTimedOut[i])) {
          // Successfully initialized
        } else {
          allSensorsInitialized = false;  // At least one sensor still pending
        }
      }
      if (sensorTimedOut[i] && !sensorInitialized[i]) {
        if (shouldPrint()) {
          Serial.print("Sensor ");
          Serial.print(getSensorName(static_cast<SensorName>(i)));
          Serial.println(" failed to initialize within timeout - proceeding with fallback.");
        }
      }
    }
    wdt_reset();  // Prevent watchdog timeout during initialization
  }

  if (!allSensorsInitialized) {
    if (shouldPrint()) {
      Serial.println("Total initialization timeout reached. Proceeding with available sensors.");
    }
  }

  printSensorHealth();  // Initial health status
}

// ====================
// Loop Function
// ====================

void loop() {
  wdt_reset();  // Reset watchdog at loop start

  unsigned long currentMillis = millis();

  // Actuator State Machine
  logStateChange();
  unsigned long elapsedTime = currentMillis - actuatorPreviousMillis;

  switch (currentState) {
    case RETRACTING_INITIAL:
      if (elapsedTime >= RETRACT_DURATION) {
        actuatorPreviousMillis = currentMillis;
        currentState = STOPPING_INITIAL;
        controlMotor(false, 0);
      } else {
        controlMotor(true, -1);  // Retract
        wdt_reset();  // Prevent reset during long retraction
      }
      break;

    case STOPPING_INITIAL:
      if (elapsedTime >= INITIAL_STOP_DURATION) {
        actuatorPreviousMillis = currentMillis;
        currentState = EXTENDING_SEQUENCE;
      } else {
        controlMotor(false, 0);  // Stop
      }
      break;

    case EXTENDING_SEQUENCE:
      wdt_reset();  // Prevent reset during long sequence
      if (sequenceCount < REPETITIONS) {
        if (elapsedTime < EXTEND_TIME) {
          controlMotor(true, 1);  // Extend
        } else if (elapsedTime < EXTEND_TIME + STOP_TIME) {
          controlMotor(false, 0);  // Stop
        } else {
          actuatorPreviousMillis = currentMillis;
          sequenceCount++;
        }
      } else {
        sequenceCount = 0;
        currentState = RETRACTING_SEQUENCE;
      }
      break;

    case RETRACTING_SEQUENCE:
      wdt_reset();  // Prevent reset during long sequence
      if (sequenceCount < REPETITIONS) {
        if (elapsedTime < RETRACT_TIME) {
          controlMotor(true, -1);  // Retract
        } else if (elapsedTime < RETRACT_TIME + STOP_TIME) {
          controlMotor(false, 0);  // Stop
        } else {
          actuatorPreviousMillis = currentMillis;
          sequenceCount++;
          if (sequenceCount >= REPETITIONS) {
            // Reset of the Arduino after the last RETRACTING_SEQUENCE
            wdt_enable(WDTO_15MS);  // Enable watchdog with 15ms timeout
            while (true) {
              // Infinite loop to trigger watchdog reset
            }
          }
        }
      } else {
        sequenceCount = 0;
        currentState = RETRACTING_INITIAL;
      }
      break;
  }

  wdt_reset();  // Reset after actuator control

  // Sensor Reading
  static bool readingInProgress = false;  // Track reading state
  if (currentMillis - previousSensorMillis >= readInterval && !readingInProgress) {
    previousSensorMillis = currentMillis;
    sensorIndex = 0;
    readingCycleComplete = false;
    readingInProgress = true;  // Start reading cycle
  }

  if (readingInProgress && sensorIndex < NUM_SENSORS && currentMillis - sensorDelayMillis >= sensorDelayInterval) {
    sensorDelayMillis = currentMillis;
    sensors_event_t humidity, temp;

    if (disabledSensors[sensorIndex]) {
      float fallbackTemp = getFallbackTemperature(sensorIndex);
      float fallbackHumidity = getFallbackHumidity(sensorIndex);
      adjustFanAndDimmer(fallbackTemp, fallbackHumidity, static_cast<SensorName>(sensorIndex));
      sensorIndex++;
    } else {
      if (readSensorData(sensorIndex + 2, sensorIndex, humidity, temp)) {  // Success
        if (sensorIndex == EXT) {
          extTemperature = temp.temperature;  // Store EXT temperature
          if (shouldPrint()) {
            Serial.print(getSensorName(static_cast<SensorName>(sensorIndex)));
            Serial.print(" - Temperature: ");
            Serial.print(temp.temperature);
            Serial.print(" °C, Humidity: ");
            Serial.print(humidity.relative_humidity);
            Serial.println(" %");
          }
        } else if (sensorIndex == TOP) {
          if (shouldPrint()) {
            Serial.print(getSensorName(static_cast<SensorName>(sensorIndex)));
            Serial.print(" - Temperature: ");
            Serial.print(temp.temperature);
            Serial.print(" °C, Humidity: ");
            Serial.print(humidity.relative_humidity);
            Serial.println(" %");
          }
          topTemperatures[readingIndex] = temp.temperature;
          topHumidities[readingIndex] = humidity.relative_humidity;
        } else if (sensorIndex == BOT_D) {
          botSensorData.temperatureD = temp.temperature;
          botSensorData.humidityD = humidity.relative_humidity;
          botSensorData.updatedD = true;
          botDTemperatures[readingIndex] = temp.temperature;
          botDHumidities[readingIndex] = humidity.relative_humidity;
        } else if (sensorIndex == BOT_G) {
          botSensorData.temperatureG = temp.temperature;
          botSensorData.humidityG = humidity.relative_humidity;
          botSensorData.updatedG = true;
          botGTemperatures[readingIndex] = temp.temperature;
          botGHumidities[readingIndex] = humidity.relative_humidity;

          if (botSensorData.updatedD && botSensorData.updatedG) {
            float avgTemp = (botSensorData.temperatureD + botSensorData.temperatureG) / 2.0;
            float avgHumidity = (botSensorData.humidityD + botSensorData.humidityG) / 2.0;
            if (shouldPrint()) {
              Serial.print("BOT-Av - Temperature: ");
              Serial.print(avgTemp);
              Serial.print(" °C, Humidity: ");
              Serial.print(avgHumidity);
              Serial.println(" %");
            }
          }
        }
        adjustFanAndDimmer(temp.temperature, humidity.relative_humidity, static_cast<SensorName>(sensorIndex));
        sensorIndex++;
      } else if (currentMillis - sensorDelayMillis >= RETRY_DELAY * MAX_I2C_RETRIES + BUS_RESET_DELAY + 100) {  // Timeout check
        float fallbackTemp = getFallbackTemperature(sensorIndex);
        float fallbackHumidity = getFallbackHumidity(sensorIndex);
        adjustFanAndDimmer(fallbackTemp, fallbackHumidity, static_cast<SensorName>(sensorIndex));
        sensorIndex++;
      }
    }

    if (sensorIndex >= NUM_SENSORS) {
      readingCycleComplete = true;
      readingInProgress = false;  // End reading cycle
    }
  }

  if (readingCycleComplete) {
    if (shouldPrint()) {
      Serial.println();
      printFanRPMs();
      printAverages();
      printHumidityAverages();  // Print humidity averages after temperature averages
    }

    printNextSequence();  // Print the next actuator sequence

    cycleCounter++;
    if (cycleCounter >= PRINT_CYCLE_INTERVAL * 10) {  // Reset after 10 intervals
      cycleCounter = 0;  // Reset cycle counter appropriately
    }

    printSensorHealth();  // Print health status every cycle
    if (shouldPrint()) {
      Serial.println();
    }
    readingCycleComplete = false;
    readingIndex = (readingIndex + 1) % N_READINGS;
  }

  wdt_reset();  // Reset after sensor reading

  // Pulse Measurement
  measurePulseNonBlocking(TACH_F1BG, 0);
  measurePulseNonBlocking(TACH_F2BD, 1);
  measurePulseNonBlocking(TACH_F3BC, 2);
  measurePulseNonBlocking(TACH_F4HG, 3);
  measurePulseNonBlocking(TACH_F5EXT, 4);
  measurePulseNonBlocking(TACH_F6BC, 5);
  measurePulseNonBlocking(TACH_F7HD, 6);

  // Check and re-enable sensors periodically
  checkDisabledSensors();
}

// ====================
// Additional Functions
// ====================

void printFanRPMs() {
  if (shouldPrint()) {
    unsigned long rpmF1BG = calculateRPMNonBlocking(0);
    unsigned long rpmF2BD = calculateRPMNonBlocking(1);
    unsigned long rpmF3BC = calculateRPMNonBlocking(2);
    unsigned long rpmF4HG = calculateRPMNonBlocking(3);
    unsigned long rpmF5EXT = calculateRPMNonBlocking(4);
    unsigned long rpmF6BC = calculateRPMNonBlocking(5);
    unsigned long rpmF7HD = calculateRPMNonBlocking(6);

    Serial.print("EXT: F5EXT: ");
    Serial.print(rpmF5EXT);
    Serial.println(" RPM");

    Serial.print("TOP: F4HG: ");
    Serial.print(rpmF4HG);
    Serial.print(" RPM,");
    Serial.print(" F7HD: ");
    Serial.print(rpmF7HD);
    Serial.println(" RPM");

    Serial.print("BOT: F1BG: ");
    Serial.print(rpmF1BG);
    Serial.print(" RPM, F2BD: ");
    Serial.print(rpmF2BD);
    Serial.print(" RPM, F3BC: ");
    Serial.print(rpmF3BC);
    Serial.print(" RPM,");
    Serial.print(" F6BC: ");
    Serial.print(rpmF6BC);
    Serial.println(" RPM"); 
    Serial.println();
    
  }
}

void adjustFanAndDimmer(float temperature, float humidity, SensorName sensorName) {
  float validTemp = isValidReading(temperature, MIN_VALID_TEMP, MAX_VALID_TEMP) ? temperature : getFallbackTemperature(sensorName);
  float validHumidity = isValidReading(humidity, MIN_VALID_HUMIDITY, MAX_VALID_HUMIDITY) ? humidity : getFallbackHumidity(sensorName);

  switch (sensorName) {
    case TOP: adjustFanAndDimmerForTop(validTemp, validHumidity); break;
    case BOT_D:
    case BOT_G: adjustFanAndDimmerForBot(validTemp, validHumidity, sensorName); break;
    case EXT: adjustFanAndDimmerForExt(validTemp, validHumidity); break;
  }
}

void adjustFanAndDimmerForTop(float temperature, float humidity) {
  if (temperature < VERY_LOW_TEMP_TOP) {
    topLight.setBrightness(percentageToPWM(DIMMER_HIGH));
    analogWrite(PWM_F4HG, percentageToPWM(FAN_HIGH));
    if (shouldPrint()) {
      Serial.println("Dimmer: High Brightness (Very Low Temperature)");
      Serial.println("Fan F4HG: High Speed (Very Low Temperature)");
    }
  } else if (temperature < LOW_TEMP_TOP) {
    topLight.setBrightness(percentageToPWM(DIMMER_LOW));
    analogWrite(PWM_F4HG, percentageToPWM(FAN_MEDIUM));
    if (shouldPrint()) {
      Serial.println("Dimmer: Low Brightness (Low Temperature)");
      Serial.println("Fan F4HG: Medium Speed (Low Temperature)");
    }
  } else if (temperature >= LOW_TEMP_TOP && temperature <= HIGH_TEMP_TOP) {
    topLight.setBrightness(percentageToPWM(DIMMER_OFF));
    analogWrite(PWM_F4HG, percentageToPWM(FAN_LOW));
    if (shouldPrint()) {
      Serial.println("Dimmer: Off (Medium Temperature)");
      Serial.println("Fan F4HG: Low Speed (Medium Temperature)");
    }
  } else if (temperature > HIGH_TEMP_TOP) {
    topLight.setBrightness(percentageToPWM(DIMMER_OFF));
    analogWrite(PWM_F4HG, percentageToPWM(FAN_HIGH));
    if (shouldPrint()) {
      Serial.println("Dimmer: Off (High Temperature)");
      Serial.println("Fan F4HG: High Speed (High Temperature)");
    }
  }

  if (humidity < VERY_LOW_HUMIDITY_TOP) {
    analogWrite(PWM_F7HD, percentageToPWM(FAN_HIGH));
    if (shouldPrint()) {
      Serial.println("Fan F7HD: High Speed (Very Low Humidity)");
    }
  } else if (humidity >= VERY_LOW_HUMIDITY_TOP && humidity < LOW_HUMIDITY_TOP) {
    analogWrite(PWM_F7HD, percentageToPWM(FAN_HIGH));
    if (shouldPrint()) {
      Serial.println("Fan F7HD: High Speed (Low Humidity)");
    }
  } else if (humidity >= LOW_HUMIDITY_TOP && humidity < MEDIUM_HUMIDITY_TOP) {
    analogWrite(PWM_F7HD, percentageToPWM(FAN_LOW));
    if (shouldPrint()) {
      Serial.println("Fan F7HD: Very Low Speed (Medium Humidity)");
    }
  } else {
    analogWrite(PWM_F7HD, percentageToPWM(FAN_OFF));
    if (shouldPrint()) {
      Serial.println("Fan F7HD: Off (High Humidity)");
    }
  }

  if (shouldPrint()) {
    Serial.println();
  }
}

void adjustFanAndDimmerForBot(float temperature, float humidity, SensorName sensorName) {
  if (sensorName == BOT_D) {
    botSensorData.temperatureD = temperature;
    botSensorData.humidityD = humidity;
    botSensorData.updatedD = true;
  } else if (sensorName == BOT_G) {
    botSensorData.temperatureG = temperature;
    botSensorData.humidityG = humidity;
    botSensorData.updatedG = true;
  }

  if (botSensorData.updatedD && botSensorData.updatedG) {
    float botTemperatureAvg = (botSensorData.temperatureD + botSensorData.temperatureG) / 2;
    float botHumidityAvg = (botSensorData.humidityD + botSensorData.humidityG) / 2;

    // Adjust dimmer thresholds if EXT temperature is below VERY_LOW_TEMP_EXT or above LOW_TEMP_EXT
    int adjustedDimmerLow = DIMMER_LOW;
    int adjustedDimmerMedium = DIMMER_MEDIUM;
    int adjustedDimmerHigh = DIMMER_HIGH;

    if (extTemperature < VERY_LOW_TEMP_EXT) {
      adjustedDimmerLow += 10;
      adjustedDimmerMedium += 10;
      adjustedDimmerHigh += 10;
      if (shouldPrint()) {
        Serial.println("Dimmer: Plus 10% due to <VERY_LOW_TEMP_EXT");
      }
    } else if (extTemperature > LOW_TEMP_EXT) {
      adjustedDimmerLow = max(0, adjustedDimmerLow - 10);
      adjustedDimmerMedium = max(0, adjustedDimmerMedium - 10);
      adjustedDimmerHigh = max(0, adjustedDimmerHigh - 10);
      if (shouldPrint()) {
        Serial.println("Dimmer: Less 10% due to >LOW_TEMP_EXT");
      }
    }

    if (botTemperatureAvg < VERY_LOW_TEMP_BOT) {
      botLight.setBrightness(percentageToPWM(adjustedDimmerHigh));
      analogWrite(PWM_F1BG_F2BD_F3BC, percentageToPWM(FAN_HIGH));
      if (shouldPrint()) {
        Serial.println("Dimmer: High Brightness (Very Low Temperature)");
        Serial.println("Fan F1BG_F2BD_F3BC: High Speed (Very Low Temperature)");
      }
    } else if (botTemperatureAvg < LOW_TEMP_BOT) {
      botLight.setBrightness(percentageToPWM(adjustedDimmerLow));
      analogWrite(PWM_F1BG_F2BD_F3BC, percentageToPWM(FAN_MEDIUM));
      if (shouldPrint()) {
        Serial.println("Dimmer: Low Brightness (Low Temperature)");
        Serial.println("Fan F1BG_F2BD_F3BC: Medium Speed (Low Temperature)");
      }
    } else if (botTemperatureAvg >= LOW_TEMP_BOT && botTemperatureAvg <= HIGH_TEMP_BOT) {
      botLight.setBrightness(percentageToPWM(DIMMER_OFF));
      analogWrite(PWM_F1BG_F2BD_F3BC, percentageToPWM(FAN_LOW));
      if (shouldPrint()) {
        Serial.println("Dimmer: Off (Medium Temperature)");
        Serial.println("Fan F1BG_F2BD_F3BC: Low Speed (Medium Temperature)");
      }
    } else if (botTemperatureAvg > HIGH_TEMP_BOT) {
      botLight.setBrightness(percentageToPWM(DIMMER_OFF));
      analogWrite(PWM_F1BG_F2BD_F3BC, percentageToPWM(FAN_HIGH));
      if (shouldPrint()) {
        Serial.println("Dimmer: Off (High Temperature)");
        Serial.println("Fan F1BG_F2BD_F3BC: High Speed (High Temperature)");
      }
    }

    if (botHumidityAvg < VERY_LOW_HUMIDITY_BOT) {
      analogWrite(PWM_F6BC, percentageToPWM(FAN_HIGH));
      if (shouldPrint()) {
        Serial.println("Fan F6BC: High Speed (Very Low Humidity)");
      }
    } else if (botHumidityAvg >= VERY_LOW_HUMIDITY_BOT && botHumidityAvg < LOW_HUMIDITY_BOT) {
      analogWrite(PWM_F6BC, percentageToPWM(FAN_MEDIUM));
      if (shouldPrint()) {
        Serial.println("Fan F6BC: Medium Speed (Low Humidity)");
      }
    } else if (botHumidityAvg >= LOW_HUMIDITY_BOT && botHumidityAvg < MEDIUM_HUMIDITY_BOT) {
      analogWrite(PWM_F6BC, percentageToPWM(FAN_LOW));
      if (shouldPrint()) {
        Serial.println("Fan F6BC: Very Low Speed (Medium Humidity)");
      }
    } else {
      analogWrite(PWM_F6BC, percentageToPWM(FAN_OFF));
      if (shouldPrint()) {
        Serial.println("Fan F6BC: Off (High Humidity)");
      }
    }

    botSensorData.updatedD = false;
    botSensorData.updatedG = false;
  }
}

void adjustFanAndDimmerForExt(float temperature, float humidity) {
  float botHumidityAvg = (botSensorData.humidityD + botSensorData.humidityG) / 2;

  if (botHumidityAvg > HIGH_HUMIDITY_BOT) {
    analogWrite(PWM_F5EXT, percentageToPWM(FAN_HIGH));
    if (shouldPrint()) {
      Serial.println("Fan F5EXT: High Speed (EXT_SENSOR override - Bot Humidity > 55.0%)");
    }
  } else {
    if (temperature > HIGH_TEMP_EXT) {
      analogWrite(PWM_F5EXT, percentageToPWM(FAN_HIGH));
      if (shouldPrint()) {
        Serial.println("Fan F5EXT: High Speed (High Temperature)");
      }
    } else {
      analogWrite(PWM_F5EXT, percentageToPWM(FAN_VERY_LOW));
      if (shouldPrint()) {
        Serial.println("Fan F5EXT: Very Low Speed (Otherwise)");
      }
    }
  }

  if (shouldPrint()) {
    Serial.println();
  }
}