#include <Arduino.h>
#include <BleGamepad.h>

#define PPM_PIN     34    // FS-i6 PPM-OUT → GPIO34
#define CHANNELS    6
#define PPM_SYNC    3000  // µs gap to detect frame sync

// --- BLE gamepad setup ---
BleGamepad bleGamepad("FS-i6 Joystick", "Dev", 100);

// --- PPM storage ---
volatile uint16_t ppmValues[CHANNELS];
volatile uint8_t  ppmIndex   = 0;
volatile unsigned long lastMic = 0;
// New: Timestamp of the last *full* PPM frame received.
volatile unsigned long lastFullPPMFrameMicros = 0; 

// --- Smoothing filter setup ---
// Define the size of the moving average window.
// A smaller window (e.g., 3-5) reduces latency significantly but might offer less smoothing.
// A larger window (e.g., 10-20) offers more smoothing but increases latency.
// Adjust this value based on your desired trade-off between responsiveness and stability.
#define FILTER_WINDOW_SIZE 5 // Reduced for lower latency
uint16_t filterBuffer[CHANNELS][FILTER_WINDOW_SIZE];
uint8_t  filterBufferIndex = 0;

// --- Calibration and Dead Zone Setup ---
// Number of samples to take for calibration at startup
#define CALIBRATION_SAMPLES 100 
// Delay between calibration samples (ms) - only affects startup, not runtime latency
#define CALIBRATION_SAMPLE_DELAY_MS 5
// Dead zone threshold for each axis (in PPM units). Values within +/- this amount from the calibrated center will be treated as center.
#define CALIBRATION_DEADZONE 25 
// Stores the calibrated center value for each of the 4 analog joystick channels
uint16_t calibratedCenters[4]; 

// --- PPM Watchdog Setup ---
// If no full PPM frame is received within this time (microseconds), assume signal loss
// and force axes to center. A typical PPM frame is 20-25ms. 50ms provides a buffer.
#define PPM_TIMEOUT_MICROS 50000 // 50 milliseconds


// ISR: called on each rising PPM edge
void IRAM_ATTR onPPM() {
  unsigned long now   = micros();
  unsigned long pulse = now - lastMic;
  lastMic = now;

  if (pulse > PPM_SYNC) {
    // Sync pulse detected, reset index for new frame
    ppmIndex = 0;
  } else if (ppmIndex < CHANNELS) {
    // Store pulse and increment index
    ppmValues[ppmIndex++] = pulse;
    // If we've just received the last channel for a full frame, update timestamp
    if (ppmIndex == CHANNELS) {
      lastFullPPMFrameMicros = now;
    }
  }
}

void setup() {
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), onPPM, RISING);

  BleGamepadConfiguration cfg;
  cfg.setAutoReport(true);
  cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  // Set to max power (9 dBm) for better range
  cfg.setTXPowerLevel(9); 
  bleGamepad.begin(&cfg);

  // --- Initializing Filter Buffer and Calibrating Joysticks ---
  Serial.begin(115200); // For debugging output during calibration

  // Give the PPM signal some time to stabilize after power-up
  delay(500); 

  // Variables to hold sum of readings for calibration
  long chSum[4] = {0}; 
  uint16_t currentPpm[CHANNELS]; // Temporary buffer for PPM values

  Serial.println("Calibrating joysticks. Please keep joysticks centered...");

  // Collect samples for calibration
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    noInterrupts();
    // Copy current PPM values. Only need first 4 for analog axes calibration.
    memcpy(currentPpm, (const void*)ppmValues, sizeof(currentPpm[0]) * CHANNELS); 
    interrupts();

    // Accumulate sums for the first 4 analog channels
    for (int j = 0; j < 4; j++) {
      chSum[j] += currentPpm[j];
    }
    delay(CALIBRATION_SAMPLE_DELAY_MS); // Small delay between samples
  }

  // Calculate and store calibrated center values
  for (int i = 0; i < 4; i++) {
    calibratedCenters[i] = chSum[i] / CALIBRATION_SAMPLES;
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(" calibrated center: ");
    Serial.println(calibratedCenters[i]);
  }
  Serial.println("Calibration complete.");

  // Initialize the filter buffer with the newly calibrated values
  // This ensures the filter starts with stable data points
  for (int i = 0; i < CHANNELS; i++) {
    for (int j = 0; j < FILTER_WINDOW_SIZE; j++) {
      // Use calibrated center for analog axes, and current PPM for others
      filterBuffer[i][j] = (i < 4) ? calibratedCenters[i] : currentPpm[i]; 
    }
  }

  // Initialize lastFullPPMFrameMicros to current time to avoid immediate timeout
  lastFullPPMFrameMicros = micros(); 
}

void loop() {
  if (bleGamepad.isConnected()) {
    uint16_t ch[CHANNELS];
    bool ppmSignalGood = true;

    // --- Check PPM Signal Health with Watchdog ---
    noInterrupts();
    unsigned long timeSinceLastFullFrame = micros() - lastFullPPMFrameMicros;
    // Check if a full PPM frame was received recently
    if (timeSinceLastFullFrame > PPM_TIMEOUT_MICROS) {
      ppmSignalGood = false; // PPM signal is stale or lost
    } else {
      // PPM signal is good, copy current values
      memcpy(ch, (const void*)ppmValues, sizeof(ch[0]) * CHANNELS);
    }
    interrupts();

    if (ppmSignalGood) {
      // Apply moving average filter
      for (int i = 0; i < CHANNELS; i++) {
        filterBuffer[i][filterBufferIndex] = ch[i]; 
        
        long sum = 0;
        for (int j = 0; j < FILTER_WINDOW_SIZE; j++) {
          sum += filterBuffer[i][j];
        }
        ch[i] = sum / FILTER_WINDOW_SIZE; 
      }
      filterBufferIndex = (filterBufferIndex + 1) % FILTER_WINDOW_SIZE; 

      // --- Apply Dead Zone to Analog Joystick Channels (0-3) ---
      for (int i = 0; i < 4; i++) { 
        if (abs((long)ch[i] - calibratedCenters[i]) < CALIBRATION_DEADZONE) {
          ch[i] = calibratedCenters[i]; 
        }
      }
    } else {
      // If PPM signal is not good, force analog axes to calibrated center
      // and buttons to released state to prevent false inputs.
      for (int i = 0; i < 4; i++) {
        ch[i] = calibratedCenters[i];
      }
      // For buttons, set them to a value that will trigger 'release'
      ch[4] = 0; // Example: assuming anything <= 1500 is release
      ch[5] = 0; // Example: assuming anything <= 1500 is release
      Serial.println("PPM signal lost or stale, forcing axes to center.");
    }

    // Calibrated min/max per channel
    const uint16_t chMin[4] = {1000, 1000, 1100, 1000};  // CH1–CH4
    const uint16_t chMax[4] = {2000, 2000, 1900, 2000};

    auto mapChannel = [](uint16_t val, uint16_t minVal, uint16_t maxVal) {
      val = constrain(val, minVal, maxVal);
      return (val - minVal) * 32767 / (maxVal - minVal);
    };

    // Joystick values
    bleGamepad.setAxes(
      mapChannel(ch[0], chMin[0], chMax[0]),  // Roll
      mapChannel(ch[1], chMin[1], chMax[1]),  // Pitch
      mapChannel(ch[2], chMin[2], chMax[2]),  // Throttle
      mapChannel(ch[3], chMin[3], chMax[3])   // Yaw
    );

    // Proper button logic
    if (ch[4] > 1500) bleGamepad.press(1); else bleGamepad.release(1);
    if (ch[5] > 1500) bleGamepad.press(2); else bleGamepad.release(2);

    bleGamepad.sendReport();
  }
}
