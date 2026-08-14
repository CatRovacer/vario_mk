/*
  RF22B Direct Mode FSK Transmitter with Sweeping Tone
  ESP32-S3 + RF22B (433.75 MHz)
  Frequency sweep: 200-1700 Hz
  Direct Mode: Data pin directly modulated by tone
*/

#include <Arduino.h>

// RF22B Pin Definitions (ESP32-S3)
#define RF22B_DATA_PIN  2    // Data input (tone signal)
#define RF22B_ENABLE_PIN 3   // TX/RX enable (HIGH = TX)
#define RF22B_CS_PIN    4    // SPI Chip Select (not used in direct mode but keep high)
#define RF22B_SDN_PIN   5    // Shutdown (LOW = active)

// RF22B Configuration - Direct Mode
#define CARRIER_FREQUENCY 433075000UL  // 433.075 MHz
#define FREQ_DEVIATION    15000        // ±15 kHz deviation (adjustable)

// Tone Parameters
#define MIN_FREQ 200.0
#define MAX_FREQ 1700.0
#define SWEEP_INTERVAL 20  // ms between frequency steps
#define SWEEP_STEP 0.5     // Hz per step

// EEPROM (using RTC memory for persistence)
RTC_DATA_ATTR float savedMinFreq = MIN_FREQ;
RTC_DATA_ATTR float savedMaxFreq = MAX_FREQ;

// Variables
float currentFreq = MIN_FREQ;
bool sweepUp = true;
uint32_t lastSweepTime = 0;
bool transmitEnabled = false;

// Function prototypes
void configureRF22B();
void generateTone(float freq);
void processSerialCommands();

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== RF22B Tone Generator ===");
  Serial.println("Direct Mode FSK - 433.75 MHz");
  Serial.printf("Sweep range: %.1f - %.1f Hz\n", savedMinFreq, savedMaxFreq);
  Serial.println("Commands:");
  Serial.println("  S<min>,<max> - Set sweep range (e.g., S300,1500)");
  Serial.println("  T - Toggle transmission ON/OFF");
  Serial.println("  F - Show current settings");
  Serial.println("  R - Reset to defaults");
  Serial.println("================================");

  // Initialize pins
  pinMode(RF22B_DATA_PIN, OUTPUT);
  pinMode(RF22B_ENABLE_PIN, OUTPUT);
  pinMode(RF22B_CS_PIN, OUTPUT);
  pinMode(RF22B_SDN_PIN, OUTPUT);
  
  digitalWrite(RF22B_DATA_PIN, LOW);
  digitalWrite(RF22B_ENABLE_PIN, LOW);  // Disable TX initially
  digitalWrite(RF22B_CS_PIN, HIGH);      // CS idle
  digitalWrite(RF22B_SDN_PIN, LOW);      // Active mode

  // Configure RF22B for 433.75 MHz Direct Mode
  configureRF22B();

  // Start with transmission enabled
  transmitEnabled = true;
  digitalWrite(RF22B_ENABLE_PIN, HIGH);
  Serial.println("Transmission started (Direct Mode)");

  currentFreq = savedMinFreq;
  lastSweepTime = millis();
}

void loop() {
  // Generate tone
  if (transmitEnabled) {
    generateTone(currentFreq);
  }

  // Update frequency sweep
  if (millis() - lastSweepTime >= SWEEP_INTERVAL) {
    lastSweepTime = millis();
    
    if (sweepUp) {
      currentFreq += SWEEP_STEP;
      if (currentFreq >= savedMaxFreq) {
        currentFreq = savedMaxFreq;
        sweepUp = false;
      }
    } else {
      currentFreq -= SWEEP_STEP;
      if (currentFreq <= savedMinFreq) {
        currentFreq = savedMinFreq;
        sweepUp = true;
      }
    }
  }

  // Process serial commands
  processSerialCommands();
}

void generateTone(float freq) {
  static uint32_t lastToggleTime = 0;
  static bool pinState = LOW;
  
  // Calculate period in microseconds
  float period = 1000000.0 / freq;
  uint32_t halfPeriod = (uint32_t)(period / 2.0);
  
  if (micros() - lastToggleTime >= halfPeriod) {
    lastToggleTime = micros();
    pinState = !pinState;
    digitalWrite(RF22B_DATA_PIN, pinState);
  }
}

void configureRF22B() {
  // Since we're using Direct Mode, we just need to set the carrier frequency
  // This would normally be done via SPI commands to the Si4432
  // For RF22B, we need to send configuration via serial or specific protocol
  
  // Note: RF22B typically requires a startup sequence:
  // 1. Power up (SDN = LOW)
  // 2. Configure via SPI (for non-direct mode) or set pins for direct mode
  
  Serial.println("Configuring RF22B in Direct Mode...");
  Serial.printf("Carrier: %.3f MHz\n", CARRIER_FREQUENCY / 1000000.0);
  Serial.printf("Deviation: ±%d kHz\n", FREQ_DEVIATION / 1000);
  Serial.println("Direct Mode: DATA pin directly modulates carrier");
  
  // In Direct Mode, the RF22B/Si4432 needs to be configured via SPI
  // This is a placeholder - actual SPI configuration would go here
  
  // For now, we'll just enable the transmitter
  // In a real implementation, you'd need to:
  // 1. Initialize SPI
  // 2. Write configuration registers
  // 3. Set frequency, deviation, etc.
  
  // Apply a small delay for the RF chip to settle
  delay(50);
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() == 0) return;
    
    char command = cmd.charAt(0);
    String params = cmd.substring(1);
    params.trim();
    
    switch(command) {
      case 'S':
      case 's': {
        // Parse min,max
        int commaIdx = params.indexOf(',');
        if (commaIdx > 0) {
          float minF = params.substring(0, commaIdx).toFloat();
          float maxF = params.substring(commaIdx + 1).toFloat();
          
          if (minF > 0 && maxF > minF && maxF <= 5000) {
            savedMinFreq = minF;
            savedMaxFreq = maxF;
            currentFreq = minF;
            Serial.printf("Sweep range updated: %.1f - %.1f Hz\n", minF, maxF);
            Serial.println("Settings saved to RTC memory");
          } else {
            Serial.println("Invalid range! Use: S<min>,<max> (min>0, max>min, max<=5000)");
          }
        } else {
          Serial.println("Format: S<min>,<max> (e.g., S200,1700)");
        }
        break;
      }
      
      case 'T':
      case 't': {
        transmitEnabled = !transmitEnabled;
        digitalWrite(RF22B_ENABLE_PIN, transmitEnabled ? HIGH : LOW);
        Serial.printf("Transmission: %s\n", transmitEnabled ? "ON" : "OFF");
        if (!transmitEnabled) {
          digitalWrite(RF22B_DATA_PIN, LOW);  // Ensure data pin is low when disabled
        }
        break;
      }
      
      case 'F':
      case 'f': {
        Serial.println("=== Current Settings ===");
        Serial.printf("Sweep Range: %.1f - %.1f Hz\n", savedMinFreq, savedMaxFreq);
        Serial.printf("Current Frequency: %.1f Hz\n", currentFreq);
        Serial.printf("Carrier: %.3f MHz\n", CARRIER_FREQUENCY / 1000000.0);
        Serial.printf("Deviation: ±%d kHz\n", FREQ_DEVIATION / 1000);
        Serial.printf("Transmission: %s\n", transmitEnabled ? "ON" : "OFF");
        Serial.println("Direct Mode: Active");
        Serial.println("========================");
        break;
      }
      
      case 'R':
      case 'r': {
        savedMinFreq = MIN_FREQ;
        savedMaxFreq = MAX_FREQ;
        currentFreq = MIN_FREQ;
        Serial.printf("Reset to defaults: %.1f - %.1f Hz\n", MIN_FREQ, MAX_FREQ);
        break;
      }
      
      default: {
        Serial.println("Unknown command. Available: S, T, F, R");
        break;
      }
    }
  }
}

// Optional: Add SPI configuration functions if needed
// The RF22B/Si4432 uses SPI for configuration
// Here's a template for the SPI initialization:
/*
#include <SPI.h>

void initSPI() {
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.setFrequency(1000000);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
}

void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(RF22B_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);  // Write command
  SPI.transfer(value);
  digitalWrite(RF22B_CS_PIN, HIGH);
}

uint8_t readRegister(uint8_t reg) {
  digitalWrite(RF22B_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);  // Read command
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(RF22B_CS_PIN, HIGH);
  return value;
}
*/