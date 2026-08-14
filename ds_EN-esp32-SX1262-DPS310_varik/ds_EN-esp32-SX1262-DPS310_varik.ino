/*
 * Variometer for Heltec LoRa V2 (ESP32 + SX1262) with DPS310 pressure sensor
 * Transmits intermittent CW tone signals via LoRa radio to PMR radio receiver
 * Emulates Brauniger IQ ONE variometer sound with adjustable RF frequency via Serial
 * Features: Kalman filter, calibration button, debug mode with simulated climb
 */

#include <Arduino.h>
//#include <heltec.h>
#include "SX126x.h"          // SX1262 driver library [citation:1]
#include <Adafruit_DPS310.h>
#include <Wire.h>

// ============================================================================
// Pin Definitions for Heltec LoRa V2
// ============================================================================
#define CALIB_BUTTON_PIN   0     // Boot button on Heltec V2 (GPIO0)
#define LED_BUILTIN        25    // Built-in LED

// ============================================================================
// SX1262 Configuration (using dj0abr library) [citation:1]
// ============================================================================
#define LORA_NSS           8
#define LORA_RST           12
#define LORA_BUSY          13
#define LORA_DIO1          14
#define LORA_SPI_SCK       9
#define LORA_SPI_MISO      11
#define LORA_SPI_MOSI      10

#define TX_OUTPUT_POWER    17          // dBm (max 22 dBm)
#define LORA_BANDWIDTH     4           // 4 = 125 kHz
#define LORA_SPREADING_FACTOR 9        // SF9 for good range
#define LORA_CODINGRATE    4           // 4/5
#define LORA_PREAMBLE_LENGTH 8
#define RF_DEFAULT_FREQ    434075000   // Default: 433.075 MHz

// ============================================================================
// Variometer Configuration
// ============================================================================
#define SAMPLE_INTERVAL_MS  50          // 20 Hz sampling rate
#define NEUTRAL_ZONE_CM_S   10          // Neutral zone in cm/s
#define CLIMB_SOUND_BASE_HZ 600         // Base frequency for climb tone
#define SINK_SOUND_BASE_HZ  600         // Base frequency for sink tone
#define MAX_SOUND_FREQ_HZ   1200        // Maximum tone frequency
#define MIN_SOUND_FREQ_HZ   200         // Minimum tone frequency
#define SINK_PERIOD_MIN_MS  500         // Minimum sink tone period
#define SINK_PERIOD_MAX_MS  1500        // Maximum sink tone period
#define MAX_CLIMB_CM_S      500         // Max climb for sound (5 m/s)

// Kalman filter parameters (integer scaling x100)
#define KALMAN_Q            10          // Process noise
#define KALMAN_R            50          // Measurement noise
#define KALMAN_INIT_P       100         // Initial covariance

// Debug simulation
#define SIMULATION_MODE     false       // Set true for debug without sensor

// ============================================================================
// DPS310 Sensor (Adafruit library) [citation:10]
// ============================================================================
Adafruit_DPS310 dps;

// ============================================================================
// SX1262 LoRa Object
// ============================================================================
SX126x lora(LORA_NSS, LORA_RST, LORA_BUSY, LORA_DIO1,
            LORA_SPI_SCK, LORA_SPI_MISO, LORA_SPI_MOSI);

// ============================================================================
// Global Variables
// ============================================================================
static int32_t altitude_base_pressure = 0;      // Calibrated base pressure (Pa*100)
static int32_t altitude_filtered = 0;           // Filtered altitude in cm
static int32_t prev_altitude = 0;               // Previous altitude for derivative
static int16_t climb_filtered = 0;              // Filtered vertical speed in cm/s
static uint32_t last_sample_time = 0;
static bool calibration_requested = false;
static bool sensor_available = false;
static int32_t sim_altitude = 0;
static float sim_time = 0.0f;
static uint32_t rf_frequency = RF_DEFAULT_FREQ;
static bool rf_frequency_changed = false;

// Kalman filter state
struct KalmanFilter {
    int32_t x;      // State (altitude) in cm
    int32_t p;      // Covariance (scaled x100)
    int32_t q;      // Process noise (scaled x100)
    int32_t r;      // Measurement noise (scaled x100)
    int32_t k;      // Kalman gain (scaled x100)
};

static KalmanFilter kalman;

// Sound state for CW transmission
static uint32_t tone_frequency = 0;             // Hz
static uint32_t tone_period_ms = 0;             // ms (for sink - on/off period)
static bool tone_continuous = false;
static uint32_t last_tone_toggle = 0;
static bool tone_on = false;

// ============================================================================
// Kalman Filter Implementation (Integer arithmetic, scaling x100)
// ============================================================================
void kalman_init(KalmanFilter* k, int32_t q, int32_t r, int32_t initial_x, int32_t initial_p) {
    k->q = q;
    k->r = r;
    k->x = initial_x;
    k->p = initial_p;
    k->k = 0;
}

int32_t kalman_update(KalmanFilter* k, int32_t measurement) {
    // Prediction
    k->p = k->p + k->q;
    
    // Update - integer arithmetic with scaling
    int32_t denominator = k->p + k->r;
    if (denominator != 0) {
        k->k = (k->p * 100) / denominator;      // Kalman gain in percent
    }
    
    // x = x + k * (measurement - x)
    int32_t error = measurement - k->x;
    k->x = k->x + (error * k->k) / 100;
    
    // p = (1 - k) * p
    k->p = (k->p * (100 - k->k)) / 100;
    
    return k->x;
}

// ============================================================================
// DPS310 Sensor Functions
// ============================================================================
bool read_sensor(int32_t* pressure, int32_t* temperature) {
    if (!sensor_available) {
        return false;
    }
    
    // Wait for new data [citation:10]
    if (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
        return false;
    }
    
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);
    
    // Pressure in Pa * 100 (integer)
    *pressure = (int32_t)(pressure_event.pressure * 100.0f * 100.0f);
    // Temperature in °C * 100
    *temperature = (int32_t)(temp_event.temperature * 100.0f);
    
    return true;
}

int32_t calculate_altitude(int32_t pressure, int32_t base_pressure) {
    if (pressure <= 0 || base_pressure <= 0) {
        return 0;
    }
    
    // Approximation: ~8.5 m per hPa (Pa/100)
    int32_t diff = base_pressure - pressure;
    return (diff * 85) / 10;    // Altitude in cm
}

// ============================================================================
// Calibration
// ============================================================================
void perform_calibration() {
    Serial.println("[CAL] Starting calibration...");
    
    if (!sensor_available && !SIMULATION_MODE) {
        Serial.println("[CAL] Sensor not available!");
        return;
    }
    
    int32_t sum_pressure = 0;
    int valid_readings = 0;
    
    // Take 20 samples for averaging
    for (int i = 0; i < 20; i++) {
        int32_t pressure, temperature;
        if (SIMULATION_MODE || read_sensor(&pressure, &temperature)) {
            sum_pressure += pressure;
            valid_readings++;
        }
        delay(20);
    }
    
    if (valid_readings > 0) {
        altitude_base_pressure = sum_pressure / valid_readings;
        Serial.printf("[CAL] Base pressure set to: %d (Pa*100)\n", altitude_base_pressure);
    } else {
        Serial.println("[CAL] No valid readings!");
        return;
    }
    
    // Reset Kalman filter
    kalman_init(&kalman, KALMAN_Q, KALMAN_R, 0, KALMAN_INIT_P);
    prev_altitude = 0;
    climb_filtered = 0;
    altitude_filtered = 0;
    
    // Blink LED to confirm calibration
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
    
    calibration_requested = false;
    Serial.println("[CAL] Calibration complete!");
}

// ============================================================================
// Sound Calculation - Brauniger IQ ONE Emulation
// Brauniger IQ ONE: climb = intermittent tone increasing in pitch, 
// sink = intermittent tone decreasing in pitch with longer periods [citation:4]
// ============================================================================
void calculate_sound_params(int16_t climb_cm_s, uint32_t* freq_hz, uint32_t* period_ms, bool* continuous) {
    int16_t abs_climb = (climb_cm_s > 0) ? climb_cm_s : -climb_cm_s;
    
    // Neutral zone - silence
    if (abs_climb < NEUTRAL_ZONE_CM_S) {
        *freq_hz = 0;
        *period_ms = 0;
        *continuous = false;
        return;
    }
    
    // Clamp maximum climb for sound
    if (abs_climb > MAX_CLIMB_CM_S) abs_climb = MAX_CLIMB_CM_S;
    
    // Both climb and sink use INTERMITTENT tone (Brauniger IQ ONE style)
    // Climb: frequency increases with rate, period decreases
    // Sink: frequency decreases with rate, period increases
    
    float climb_m_s = abs_climb / 100.0f;
    if (climb_m_s < 0.5f) climb_m_s = 0.5f;
    if (climb_m_s > 5.0f) climb_m_s = 5.0f;
    
    if (climb_cm_s > 0) {
        // CLIMB: Rising pitch, shorter periods
        // Frequency: 600-1200 Hz (increases with climb)
        *freq_hz = CLIMB_SOUND_BASE_HZ + (uint32_t)((climb_m_s - 0.5f) * 133.33f);
        if (*freq_hz > MAX_SOUND_FREQ_HZ) *freq_hz = MAX_SOUND_FREQ_HZ;
        if (*freq_hz < CLIMB_SOUND_BASE_HZ) *freq_hz = CLIMB_SOUND_BASE_HZ;
        
        // Period: 500-250 ms (decreases with climb)
        *period_ms = 500 - (uint32_t)((climb_m_s - 0.5f) * 55.55f);
        if (*period_ms < 250) *period_ms = 250;
        if (*period_ms > 500) *period_ms = 500;
        
        *continuous = false;
    } else {
        // SINK: Falling pitch, longer periods
        // Frequency: 600-200 Hz (decreases with sink)
        *freq_hz = SINK_SOUND_BASE_HZ - (uint32_t)((climb_m_s - 0.5f) * 88.88f);
        if (*freq_hz < MIN_SOUND_FREQ_HZ) *freq_hz = MIN_SOUND_FREQ_HZ;
        if (*freq_hz > SINK_SOUND_BASE_HZ) *freq_hz = SINK_SOUND_BASE_HZ;
        
        // Period: 500-1500 ms (increases with sink)
        *period_ms = SINK_PERIOD_MIN_MS + (uint32_t)((climb_m_s - 0.5f) * 222.22f);
        if (*period_ms > SINK_PERIOD_MAX_MS) *period_ms = SINK_PERIOD_MAX_MS;
        if (*period_ms < SINK_PERIOD_MIN_MS) *period_ms = SINK_PERIOD_MIN_MS;
        
        *continuous = false;
    }
}

// ============================================================================
// CW Tone Transmission via SX1262
// ============================================================================
void transmit_cw_tone(uint32_t frequency_hz, bool enable) {
    if (enable && frequency_hz > 0) {
        // Transmit continuous wave (CW) carrier at RF frequency
        // The tone is represented by carrier presence (ON/OFF keying)
        // SX126x driver uses SetTxContinuousWave for CW [citation:1]
        lora.SetTxContinuousWave(rf_frequency, TX_OUTPUT_POWER);
    } else {
        // Turn off transmission
        lora.Sleep();
    }
}

// ============================================================================
// Update Sound Based on Climb Rate (Intermittent Tone)
// ============================================================================
void update_sound(int16_t climb_cm_s) {
    uint32_t freq_hz = 0;
    uint32_t period_ms = 0;
    bool continuous = false;
    
    calculate_sound_params(climb_cm_s, &freq_hz, &period_ms, &continuous);
    
    tone_frequency = freq_hz;
    tone_period_ms = period_ms;
    tone_continuous = continuous;
    
    if (freq_hz == 0) {
        // Silence - turn off RF
        lora.Sleep();
        tone_on = false;
        return;
    }
    
    // Intermittent tone - toggle RF carrier ON/OFF
    uint32_t now = millis();
    uint32_t half_period = period_ms / 2;
    
    if (now - last_tone_toggle >= half_period) {
        tone_on = !tone_on;
        last_tone_toggle = now;
        
        if (tone_on) {
            // Carrier ON = tone heard
            lora.SetTxContinuousWave(rf_frequency, TX_OUTPUT_POWER);
        } else {
            // Carrier OFF = silence
            lora.Sleep();
        }
    }
}

// ============================================================================
// Process Variometer Data
// ============================================================================
void process_vario_data() {
    uint32_t now = millis();
    uint32_t dt = now - last_sample_time;
    
    if (dt < SAMPLE_INTERVAL_MS) return;
    last_sample_time = now;
    
    int32_t pressure = 0;
    int32_t temperature = 0;
    int32_t altitude = 0;
    
    // Read sensor or simulate
    if (SIMULATION_MODE) {
        // Simulate altitude change (sine wave for testing)
        sim_time += dt / 1000.0f;
        sim_altitude = 5000 + (int32_t)(2000.0f * sin(sim_time * 0.5f));
        altitude = sim_altitude;
        // Serial.printf("[SIM] Altitude: %d cm\n", altitude);
    } else {
        if (!read_sensor(&pressure, &temperature)) {
            return;
        }
        
        // If not calibrated, use current pressure as base
        if (altitude_base_pressure == 0) {
            altitude_base_pressure = pressure;
            Serial.printf("[SENSOR] Base pressure set to: %d\n", pressure);
        }
        
        altitude = calculate_altitude(pressure, altitude_base_pressure);
    }
    
    // Apply Kalman filter
    int32_t filtered_alt = kalman_update(&kalman, altitude);
    altitude_filtered = filtered_alt;
    
    // Calculate climb rate (derivative)
    int32_t delta_alt = filtered_alt - prev_altitude;
    float dt_sec = dt / 1000.0f;
    if (dt_sec > 0.1f) dt_sec = 0.1f;
    int16_t climb = (int16_t)(delta_alt / dt_sec);
    
    // Additional filtering for climb rate (moving average)
    climb_filtered = (climb_filtered * 7 + climb) / 8;
    
    prev_altitude = filtered_alt;
    
    // Update sound based on climb rate
    update_sound(climb_filtered);
    
    // Debug output
    static uint32_t last_debug = 0;
    if (now - last_debug > 1000) {
        last_debug = now;
        Serial.printf("[VARIO] Alt: %d cm | Climb: %d cm/s | Tone: %d Hz | Period: %d ms | RF: %d Hz\n",
                      filtered_alt, climb_filtered, tone_frequency, tone_period_ms, rf_frequency);
    }
}

// ============================================================================
// Set RF Frequency from Serial Input
// ============================================================================
void set_rf_frequency(uint32_t freq_hz) {
    if (freq_hz >= 430000000 && freq_hz <= 440000000) {
        rf_frequency = freq_hz;
        rf_frequency_changed = true;
        Serial.printf("[RF] Frequency set to %d Hz (%.3f MHz)\n", rf_frequency, rf_frequency/1000000.0f);
        
        // Reinitialize LoRa with new frequency
        lora.Sleep();
        int err = lora.begin(
            SX126X_PACKET_TYPE_LORA,
            rf_frequency,
            TX_OUTPUT_POWER,
            SX126X_DIO3_OUTPUT_1_8     // TCXO VDD via DIO3
        );
        if (err != 0) {
            Serial.printf("[RF] Re-init error: %d\n", err);
        } else {
            lora.LoRaConfig(
                LORA_SPREADING_FACTOR,
                LORA_BANDWIDTH,
                LORA_CODINGRATE,
                LORA_PREAMBLE_LENGTH,
                true,   // CRC on
                false   // invertIQ
            );
            lora.Sleep();
            Serial.println("[RF] LoRa reinitialized with new frequency");
        }
    } else {
        Serial.println("[RF] Error: Frequency must be in 430-440 MHz range");
        Serial.println("[RF] Usage: F433075000 for 433.075 MHz");
    }
}

// ============================================================================
// Handle Serial Commands
// ============================================================================
void handle_serial_commands() {
    if (!Serial.available()) return;
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() == 0) return;
    
    if (cmd == "C" || cmd == "c") {
        calibration_requested = true;
        Serial.println("[CMD] Calibration requested");
    }
    else if (cmd == "R" || cmd == "r") {
        Serial.println("[CMD] Resetting device...");
        ESP.restart();
    }
    else if (cmd.startsWith("F") || cmd.startsWith("f")) {
        // Format: F433075000
        String freqStr = cmd.substring(1);
        uint32_t freq = freqStr.toInt();
        set_rf_frequency(freq);
    }
    else if (cmd == "?" || cmd == "help") {
        Serial.println("\n=== Commands ===");
        Serial.println("  C       - Calibrate sensor");
        Serial.println("  R       - Restart device");
        Serial.println("  F<freq> - Set RF frequency (Hz), e.g., F433075000");
        Serial.println("  ? / help- Show this help");
        Serial.printf("  Current RF: %d Hz (%.3f MHz)\n", rf_frequency, rf_frequency/1000000.0f);
        Serial.println("================\n");
    }
    else {
        Serial.printf("[CMD] Unknown command: %s (type ? for help)\n", cmd.c_str());
    }
}

// ============================================================================
// Button Handler for Calibration
// ============================================================================
void handle_calibration_button() {
    static uint32_t last_button_time = 0;
    uint32_t now = millis();
    
    // Debounce
    if (now - last_button_time < 200) return;
    last_button_time = now;
    
    if (digitalRead(CALIB_BUTTON_PIN) == LOW) {
        calibration_requested = true;
        Serial.println("[BTN] Calibration button pressed");
    }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("LoRa Variometer - Heltec V2 + DPS310");
    Serial.println("Brauniger IQ ONE Sound Emulation (Intermittent Tone)");
    Serial.printf("Default Frequency: %.3f MHz (CW)\n", RF_DEFAULT_FREQ/1000000.0f);
    Serial.println("========================================\n");
    
    // Initialize LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Initialize button
    pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize Heltec board (display disabled to save power)
    Heltec.begin(false /*DisplayEnable*/, false /*LoRaEnable*/, true /*SerialEnable*/, true /*PABOOSTEnable*/);
    
    Serial.println("[INIT] Heltec board initialized");
    
    // Initialize SX1262 LoRa module [citation:1]
    Serial.println("[INIT] Initializing SX1262...");
    int err = lora.begin(
        SX126X_PACKET_TYPE_LORA,   // LoRa mode
        rf_frequency,              // frequency in Hz
        TX_OUTPUT_POWER,           // tx power in dBm
        SX126X_DIO3_OUTPUT_1_8     // TCXO VDD via DIO3 (Heltec V2 uses TCXO)
    );
    if (err != 0) {
        Serial.printf("[INIT] lora.begin error: %d\n", err);
        Serial.println("[INIT] Check wiring! Continuing without RF...");
    } else {
        Serial.println("[INIT] SX1262 initialized successfully");
        lora.LoRaConfig(
            LORA_SPREADING_FACTOR,
            LORA_BANDWIDTH,
            LORA_CODINGRATE,
            LORA_PREAMBLE_LENGTH,
            true,   // CRC on
            false   // invertIQ
        );
        lora.Sleep();  // Start in sleep mode
        Serial.println("[INIT] LoRa configured and in sleep mode");
    }
    
    // Initialize DPS310 sensor [citation:10]
    Wire.begin();
    if (!dps.begin_I2C(DPS310_I2CADDR_DEFAULT)) {
        Serial.println("[WARN] DPS310 not found! Check wiring.");
        sensor_available = false;
    } else {
        sensor_available = true;
        Serial.println("[INIT] DPS310 sensor found");
        
        // Configure sensor for good precision
        dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
        dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
        dps.setMode(DPS310_CONT_PRESTEMP);  // Continuous pressure + temp mode
        Serial.println("[INIT] DPS310 configured (64Hz, 64 samples, continuous mode)");
    }
    
    // Initialize Kalman filter
    kalman_init(&kalman, KALMAN_Q, KALMAN_R, 0, KALMAN_INIT_P);
    
    // Perform initial calibration if sensor available
    if (sensor_available) {
        delay(500);
        perform_calibration();
    } else if (SIMULATION_MODE) {
        Serial.println("[INIT] Running in SIMULATION mode");
        altitude_base_pressure = 101325 * 100;  // 1013.25 hPa * 100
        kalman_init(&kalman, KALMAN_Q, KALMAN_R, 5000, KALMAN_INIT_P);
    }
    
    last_sample_time = millis();
    last_tone_toggle = millis();
    
    Serial.println("\n[INIT] System ready!");
    Serial.println("Commands (type ? for help):");
    Serial.println("  C    - Calibrate");
    Serial.println("  R    - Restart");
    Serial.println("  F<Hz>- Set RF frequency, e.g., F433075000");
    Serial.println("  ?    - Help\n");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    // Handle calibration button
    handle_calibration_button();
    
    // Handle calibration request
    if (calibration_requested) {
        perform_calibration();
    }
    
    // Handle serial commands (including RF frequency change)
    handle_serial_commands();
    
    // If RF frequency was changed, update the transmission
    if (rf_frequency_changed) {
        rf_frequency_changed = false;
        // Sound will use new frequency automatically
    }
    
    // Process variometer data
    process_vario_data();
    
    // LED heartbeat - indicate activity
    static uint32_t last_heartbeat = 0;
    if (millis() - last_heartbeat > 1000) {
        last_heartbeat = millis();
        if (abs(climb_filtered) > NEUTRAL_ZONE_CM_S) {
            // Fast blink when climbing or sinking
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        } else {
            // Solid on when in neutral zone
            digitalWrite(LED_BUILTIN, HIGH);
        }
    }
    
    // Small delay for stability
    delay(5);
}
