/*
  Sounding Variometer for Arduino Nano BLE Sense
  Emulates Brauniger IQ ONE audio characteristics
  Features: Kalman filter, altitude calibration button, Bluetooth Serial
*/

//#include <PDM.h>
#include <Arduino_LPS22HB.h>
#include <ArduinoBLE.h>
const char *devTitle = "VgtableVarik";
// ==================== AUDIO PARAMETERS (Brauniger IQ ONE style) ====================
#define AUDIO_SILENT         0     // No beep (neutral/light sink)
#define AUDIO_LOW_SINK       1     // Low frequency, long pulses (sinking)
#define AUDIO_LOW_CLIMB      2     // Low frequency, short pulses (weak climb)
#define AUDIO_MEDIUM_CLIMB   3     // Medium frequency (moderate climb)
#define AUDIO_STRONG_CLIMB   4     // High frequency (strong climb)
#define AUDIO_VERY_STRONG    5     // Very high frequency (very strong climb)

// Tone frequencies (Hz) - Brauniger style [citation:1]
const int climbFrequencies[] = {0, 800, 1200, 1600, 2000, 2400};
// Pulse durations (ms) - shorter for stronger climbs
const int pulseDurations[] = {0, 300, 200, 120, 70, 40};
// Pause between pulses (ms)
const int pauseDuration = 100;

// ==================== VARIOMETER PARAMETERS ====================
#define ALT_BUFFER_SIZE      10    // For initial calibration averaging
#define SAMPLE_INTERVAL      50    // Sample every 50ms (20Hz)
#define AUDIO_UPDATE_INTERVAL 100   // Update audio every 100ms

// Audio thresholds (m/s) - configurable like Brauniger [citation:1]
float audioThresholds[] = {0.2, 0.5, 1.0, 2.0, 4.0};  // Climb rates for different tones

// ==================== KALMAN FILTER ====================
// Simple Kalman filter for altitude smoothing
struct KalmanFilter {
  float Q;      // Process noise covariance (trust model)
  float R;      // Measurement noise covariance (trust sensor)
  float P;      // Estimation error covariance
  float K;      // Kalman gain
  float X;      // State estimate (filtered altitude)
  
  void init(float initial_altitude, float process_noise = 0.01, float measurement_noise = 0.5) {
    Q = process_noise;
    R = measurement_noise;
    P = 1.0;
    X = initial_altitude;
  }
  
  float update(float measurement) {
    // Prediction update
    P = P + Q;
    
    // Kalman gain
    K = P / (P + R);
    
    // Measurement update
    X = X + K * (measurement - X);
    P = (1 - K) * P;
    
    return X;
  }
};

KalmanFilter kalmanFilter;

// ==================== GLOBAL VARIABLES ====================
float referencePressure = 1013.25;   // Reference pressure for altitude (hPa)
float currentAltitude = 0.0;         // Current altitude (m)
float verticalSpeed = 0.0;           // Vertical speed (m/s)
float filteredVerticalSpeed = 0.0;   // Kalman-filtered vertical speed
int currentAudioState = AUDIO_SILENT;
unsigned long lastSampleTime = 0;
unsigned long lastAudioUpdateTime = 0;
float altitudeBuffer[ALT_BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// Button debouncing
const int calibrationPin = 2;         // Button between pin 2 and GND
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Bluetooth Service
BLEService varioService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEFloatCharacteristic altitudeChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic varioChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic audioStateChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// ==================== AUDIO GENERATION ====================
// Generate beep tone using onboard LED (for testing) or piezo on pin 6
// For actual sound, connect a piezo buzzer to pin 6 and GND
const int buzzerPin = 6;

void playTone(int frequency, int duration) {
  if (frequency == 0 || duration == 0) {
    delay(duration);
    return;
  }
  
  // Use tone() function for piezo buzzer
  tone(buzzerPin, frequency, duration);
  
  // Visual feedback via RGB LED
  if (frequency > 0) {
    // Pulse LED based on frequency
    if (frequency < 1200) {
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, HIGH);
    } else if (frequency < 1800) {
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDG, LOW);
      digitalWrite(LEDB, HIGH);
    } else {
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, LOW);
    }
    delay(duration);
    // Turn off LED
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  } else {
    delay(duration);
  }
}

void updateAudioByClimbRate(float climbRate) {
  int newState = AUDIO_SILENT;
  float absClimb = abs(climbRate);
  
  // Sink detection (negative climb)
  if (climbRate < -audioThresholds[0]) {
    newState = AUDIO_LOW_SINK;
  }
  // Climb detection
  else if (climbRate > audioThresholds[4]) {
    newState = AUDIO_VERY_STRONG;
  }
  else if (climbRate > audioThresholds[3]) {
    newState = AUDIO_STRONG_CLIMB;
  }
  else if (climbRate > audioThresholds[2]) {
    newState = AUDIO_MEDIUM_CLIMB;
  }
  else if (climbRate > audioThresholds[1]) {
    newState = AUDIO_LOW_CLIMB;
  }
  else if (climbRate > audioThresholds[0]) {
    newState = AUDIO_LOW_CLIMB;
  }
  
  // Play tone if state changed or periodically for continuous beep
  if (newState != currentAudioState || millis() - lastAudioUpdateTime >= AUDIO_UPDATE_INTERVAL) {
    currentAudioState = newState;
    
    if (currentAudioState != AUDIO_SILENT) {
      int freq = climbFrequencies[currentAudioState];
      int duration = pulseDurations[currentAudioState];
      
      // For sink, invert the psychology (lower frequency for faster sink)
      if (currentAudioState == AUDIO_LOW_SINK) {
        freq = max(400, 800 - (int)(abs(climbRate) * 100));
        duration = min(400, (int)(abs(climbRate) * 80));
      }
      
      playTone(freq, duration);
      delay(pauseDuration);
    }
    
    lastAudioUpdateTime = millis();
  }
}

// ==================== ALTITUDE AND VARIOMETER ====================
float calculateAltitude(float pressure_hPa) {
  // International barometric formula
  // Altitude = 44330 * (1 - (P/P0)^(1/5.255))
  return 44330.0 * (1.0 - pow(pressure_hPa / referencePressure, 0.19029));
}

void calibrateAltitude() {
  Serial.println("Calibrating altitude...");
  
  // Read multiple samples to get stable reference pressure
  float sumPressure = 0;
  for (int i = 0; i < 50; i++) {
    if (BARO.readPressure()) {
      sumPressure += BARO.readPressure(MILLIBAR);
    }
    delay(10);
  }
  
  referencePressure = sumPressure / 50.0;
  
  // Calculate current altitude from calibrated reference
  float currentPress = BARO.readPressure(MILLIBAR);
  currentAltitude = calculateAltitude(currentPress);
  
  // Reset Kalman filter with current altitude
  kalmanFilter.init(currentAltitude, 0.05, 0.3);
  
  Serial.print("Reference pressure set to: ");
  Serial.print(referencePressure);
  Serial.println(" hPa");
  Serial.print("Current altitude: ");
  Serial.print(currentAltitude);
  Serial.println(" m");
  
  // Feedback: 3 quick beeps on calibration
  playTone(1000, 50);
  delay(50);
  playTone(1200, 50);
  delay(50);
  playTone(1500, 100);
}

void updateVario() {
  // Read pressure sensor
  if (BARO.readPressure()) {
    float pressure = BARO.readPressure(MILLIBAR);
    float rawAltitude = calculateAltitude(pressure);
    
    // Apply Kalman filter to altitude
    float filteredAltitude = kalmanFilter.update(rawAltitude);
    
    // Calculate vertical speed (m/s) - derivative of altitude
    static float lastAltitude = 0;
    static unsigned long lastTime = 0;
    
    if (lastTime > 0) {
      float deltaAlt = filteredAltitude - lastAltitude;
      float deltaTime = (millis() - lastTime) / 1000.0;
      
      if (deltaTime > 0) {
        verticalSpeed = deltaAlt / deltaTime;
        
        // Apply additional moving average to vertical speed
        static float speedBuffer[5] = {0};
        static int speedIndex = 0;
        speedBuffer[speedIndex] = verticalSpeed;
        speedIndex = (speedIndex + 1) % 5;
        
        float sum = 0;
        for (int i = 0; i < 5; i++) sum += speedBuffer[i];
        filteredVerticalSpeed = sum / 5.0;
      }
    }
    
    lastAltitude = filteredAltitude;
    lastTime = millis();
    currentAltitude = filteredAltitude;
  }
}

// ==================== BLUETOOTH SETUP ====================
void setupBluetooth() {
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    return;
  }
  
  BLE.setLocalName(devTitle); //("VgtableVarik");
  BLE.setAdvertisedService(varioService);
  
  varioService.addCharacteristic(altitudeChar);
  varioService.addCharacteristic(varioChar);
  varioService.addCharacteristic(audioStateChar);
  
  BLE.addService(varioService);
  
  altitudeChar.writeValue(0.0);
  varioChar.writeValue(0.0);
  audioStateChar.writeValue(0);
  
  BLE.advertise();
  
  Serial.print("Bluetooth active - device name: "); Serial.println(devTitle);
}

void updateBluetooth() {
  altitudeChar.writeValue(currentAltitude);
  varioChar.writeValue(filteredVerticalSpeed);
  audioStateChar.writeValue(currentAudioState);
  
  // Also send via Serial for debugging/monitoring
  Serial.print("Alt:");
  Serial.print(currentAltitude, 2);
  Serial.print(" m | VS:");
  Serial.print(filteredVerticalSpeed, 2);
  Serial.print(" m/s | State:");
  Serial.println(currentAudioState);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("Brauniger IQ ONE Style Variometer");
  Serial.println("==================================");
  
  // Initialize LED pins
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  
  // Initialize buzzer pin
  pinMode(buzzerPin, OUTPUT);
  
  // Initialize button
  pinMode(calibrationPin, INPUT_PULLUP);
  
  // Initialize pressure sensor [citation:6][citation:9]
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
  
  // Set sensor to highest output rate (75Hz) for better responsiveness [citation:3]
  // BARO.setOutputRate(5);  // 75Hz output rate
  
  Serial.println("Pressure sensor initialized");
  
  // Initialize PDM microphone (not used for vario but can be used for audio input)
  /*
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to initialize microphone!");
  } else {
    PDM.setGain(20);
    Serial.println("Microphone initialized");
  }
  */
  // Initial calibration
  delay(1000);
  calibrateAltitude();
  
  // Setup Bluetooth
  setupBluetooth();
  
  // Startup tone sequence
  playTone(800, 100);
  delay(100);
  playTone(1200, 100);
  delay(100);
  playTone(1600, 200);
  
  Serial.println("Ready!");
}

// ==================== LOOP ====================
void loop() {
  // Handle button press for calibration
  int reading = digitalRead(calibrationPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      calibrateAltitude();
    }
  }
  lastButtonState = reading;
  
  // Update variometer data at regular intervals
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL) {
    updateVario();
    lastSampleTime = millis();
  }
  
  // Update audio based on vertical speed
  updateAudioByClimbRate(filteredVerticalSpeed);
  
  // Update Bluetooth and Serial output (every 500ms to avoid flooding)
  static unsigned long lastBLEUpdate = 0;
  if (millis() - lastBLEUpdate >= 500) {
    updateBluetooth();
    lastBLEUpdate = millis();
  }
  
  // Handle BLE central connections
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    while (central.connected()) {
      // Update data while connected
      if (millis() - lastSampleTime >= SAMPLE_INTERVAL) {
        updateVario();
        lastSampleTime = millis();
      }
      updateBluetooth();
      updateAudioByClimbRate(filteredVerticalSpeed);
      delay(100);
    }
    Serial.println("Disconnected");
  }
  
  delay(10);
}
