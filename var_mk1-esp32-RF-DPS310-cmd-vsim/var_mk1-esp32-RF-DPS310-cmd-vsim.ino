#include <Arduino.h>

#include <Adafruit_DPS310.h>
//#include <Dps310.h>

#define VERBOSE_ENABLED false

#define RF22B_ENABLED       false //   true
#define SX1262_ENABLED        false //true
#if SX1262_ENABLED || RF22B_ENABLED
#include <Wire.h>
#include <RadioLib.h>
//#include "SX126x.h"
#endif

#define ADAFRUIT_DPS310_LIB true
#define DPS310_LIB      false

#ifdef VERBOSE_ENABLED
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif

// ============================================================================
// Конфигурация
// ============================================================================
#define SAMPLE_INTERVAL_MS    50      // 20 Гц
#define CALIB_BUTTON_PIN      0       // Кнопка калибровки (GPIO0)
#define LED_BUILTIN           25      // Встроенный LED
#define PIN_BUZZER            15
// Параметры CW-передачи
#define CW_FREQUENCY          434075000ULL  // 433.075 МГц
#define CW_POWER              22           // 22 дБм (макс.)
#define CW_BEEP_MS            30           // Длительность "бип" в мс

// Параметры фильтра Калмана (целочисленные)
#define KALMAN_Q              10           // Шум процесса (x100)
#define KALMAN_R              50           // Шум измерения (x100)
#define KALMAN_INIT_P         100          // Начальная ковариация (x100)

// Параметры звука Brauniger IQ ONE
#define SOUND_NEUTRAL_ZONE    10           // см/с - нейтральная зона
#define SOUND_MIN_FREQ        200          // Гц
#define SOUND_MAX_FREQ        1200         // Гц
#define SOUND_CLIMB_BASE      600          // Гц
#define SOUND_SINK_BASE       600          // Гц

#define MIN_BEEP_MS           50   // Минимальный период звучания (мс) (при быстром подъёме)
#define MAX_BEEP_MS           800  // Максимальный период (мс) (при быстром снижении)

#define SIMULATION_ENABLED    true    // Режим эмуляции для отладки
#define SIMULATION_PERIOD_MS  1000    // Период изменения скорости при эмуляции

/*
    Sample configurations, use the settings for your specific hardware

    // Heltec Board
    #define MOSI 10
    #define MISO 11
    #define SCK 9

    // EBYTE E220 400M22S on WROOM32
    #define MOSI 23
    #define MISO 19
    #define SCK 18
*/

// ============================================================================
// Пины для SX1262 на Heltec LoRa V2
// ============================================================================
#define LORA_NSS           8
#define LORA_CS            LORA_NSS
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
#define RF_DEFAULT_FREQ    434075000   // Default: 434.075 MHz

// ============================================================================
// Пины для RF22B
// ============================================================================
#define RF_CS               8
#define RF_DIO1             14
#define RF_RST              12
#define RF_BUSY             13

#if 0
  #define SDA_PIN 12 
  #define SCL_PIN 13 
#else
#endif

// ============================================================================
// Глобальные переменные
// ============================================================================

typedef struct {
    int32_t x;             // Состояние (высота) в см
    int32_t p;             // Ковариация (x100)
    int32_t q;             // Шум процесса (x100)
    int32_t r;             // Шум измерения (x100)
    int32_t k;             // Коэффициент Калмана (x100)
} kalman_filter_t;

typedef struct {
    int32_t pressure;      // Давление в Па (x100)
    int32_t temperature;   // Температура в °C (x100)
    int32_t altitude;      // Высота в см
    int16_t climb;         // Вертикальная скорость в см/с
} vario_data_t;

//static vario_data_t g_vario_data;

static kalman_filter_t g_kalman;
static int32_t g_altitude_base = 0;      // Базовое давление для калибровки (Па * 100)
static int32_t g_altitude_filtered = 0;  // Отфильтрованная высота (см)
static int16_t g_climb_filtered = 0;     // Отфильтрованная скорость (см/с)
static int32_t g_prev_altitude = 0;
static uint32_t g_last_sample_time = 0;
static bool g_calibration_requested = false;
static bool g_debug_mode = false;        // Режим эмуляции для отладки
static int16_t g_debug_climb = 0;        // Эмулируемая скорость
static unsigned long g_debug_last_toggle = 0;
static bool g_debug_climb_direction = true;
static unsigned long dt;

// ============================================================================
// Фильтр Калмана (целочисленная реализация, масштабирование x100)
// ============================================================================
void kalman_init(kalman_filter_t *k, int32_t q, int32_t r, int32_t initial_x, int32_t initial_p) {
    k->q = q;
    k->r = r;
    k->x = initial_x;
    k->p = initial_p;
    k->k = 0;
}

int32_t kalman_update(kalman_filter_t *k, int32_t measurement) {
    // Прогноз
    k->p = k->p + k->q;
    
    // Обновление (целочисленная арифметика)
    // k = p / (p + r)
    int32_t denominator = k->p + k->r;
    if (denominator != 0) {
        k->k = (k->p * 100) / denominator;  // Коэффициент в процентах
    }
    
    // x = x + k * (measurement - x)
    int32_t error = measurement - k->x;
    k->x = k->x + (error * k->k) / 100;
    
    // p = (1 - k) * p
    k->p = (k->p * (100 - k->k)) / 100;
    
    return k->x;
}

// ============================================================================
// Sensor Adafruit_DPS310
// ============================================================================
#if ADAFRUIT_DPS310_LIB
Adafruit_DPS310 dps;

bool read_dps310(int32_t *pressure, int32_t *temperature) {
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);
    
    if (isnan(pressure_event.pressure) || isnan(temp_event.temperature)) {
        return false;
    }
    
    // Давление в Па * 100 (для целочисленных операций)
    *pressure = (int32_t)(pressure_event.pressure * 100.0f * 100.0f);
    *temperature = (int32_t)(temp_event.temperature * 100.0f);
    
    return true;
}
#elif DPS310_LIB
// ============================================================================
// DPS310
// ============================================================================

static Dps310 g_dps310 = Dps310();

void init_sensor(void){
    g_dps310.setTempScale(DPS310_TEMP_SCALE_2);
    g_dps310.setPressScale(DPS310_PRESS_SCALE_2);
    g_dps310.setTempRate(DPS310_TEMP_RATE_4);
    g_dps310.setPressRate(DPS310_PRESS_RATE_4);
    g_dps310.setMode(DPS310_MODE_CONT);
}

bool read_sensor(int32_t *pressure, int32_t *temperature) {
    int32_t raw_pressure, raw_temperature;
    
    if (g_dps310.measureBoth() != 0) {
        return false;
    }
    
    delay(100);
    
    if (g_dps310.getPressureAndTemperature(raw_pressure, raw_temperature) != 0) {
        return false;
    }
    
    // Давление: raw_pressure в Па * 100 (для целочисленной арифметики)
    *pressure = raw_pressure;
    *temperature = raw_temperature; // °C * 100
    
    return true;
}
#endif

int32_t calculate_altitude(int32_t pressure, int32_t base_pressure) {
    if (pressure <= 0 || base_pressure <= 0) {
        return 0;
    }
    
    // Упрощенное приближение: 8.5 м на 1 гПа
    int32_t diff = base_pressure - pressure;
    return (diff * 85) / 10;  // Высота в см
}
int getBeepDuration(float climbRate) {
  // Скорость набора (положительная)
  if (climbRate > 0) {
    // Экспоненциальное уменьшение паузы: от 500ms (0.3 м/с) до 50ms (5+ м/с)
    int duration = map(constrain(climbRate, 0.3, 5.0) * 100, 30, 500, 500, 80);
    return constrain(duration, 50, 500);
  }
  // Скорость снижения (отрицательная)
  else if (climbRate < 0) {
    // При снижении импульсы длиннее и неприятнее
    int duration = map(abs(constrain(climbRate, -5.0, -0.3) * 100), 30, 500, 800, 300);
    return constrain(duration, 200, 800);
  }
  return 1000; // Молчание
}

// ============================================================================
// Генерация звуковых параметров Brauniger IQ ONE
// ============================================================================
void calculate_sound_params(int16_t climb_cm_s, int32_t *freq_hz, int32_t *period_ms, bool *continuous) {
    int16_t abs_climb = (climb_cm_s > 0) ? climb_cm_s : -climb_cm_s;
    float lastFreq = NEUTRAL_FREQ;
    float lastBeepDuration = MAX_BEEP_MS;
    unsigned long beepStartTime = 0;

    // Нейтральная зона
    if (abs_climb < SOUND_NEUTRAL_ZONE) {
        *freq_hz = 0;
        *period_ms = 0;
        *continuous = false;
        return;
    }
    
    // Ограничиваем максимальную скорость для звука
    if (abs_climb > 500) abs_climb = 500;  // 5 м/с
    
    if (climb_cm_s > 0) {
        // ПОДЪЕМ: непрерывный тон, частота растет
        float climb_m_s = abs_climb / 100.0f;
        if (climb_m_s < 0.5f) climb_m_s = 0.5f;
        if (climb_m_s > 5.0f) climb_m_s = 5.0f;
        
        *freq_hz = SOUND_CLIMB_BASE + (int32_t)((climb_m_s - 0.5f) * 133.33f);
        if (*freq_hz > SOUND_MAX_FREQ) *freq_hz = SOUND_MAX_FREQ;
        if (*freq_hz < SOUND_CLIMB_BASE) *freq_hz = SOUND_CLIMB_BASE;
        
        *period_ms = 0;
        *continuous = true;
    } else {
        // СНИЖЕНИЕ: прерывистый тон частота 200-600 Гц
        float climb_m_s = abs_climb / 100.0f;
        if (climb_m_s < 0.5f) climb_m_s = 0.5f;
        if (climb_m_s > 5.0f) climb_m_s = 5.0f;
        
        *freq_hz = SOUND_SINK_BASE - (int32_t)((climb_m_s - 0.5f) * 88.88f);
        if (*freq_hz < SOUND_MIN_FREQ) *freq_hz = SOUND_MIN_FREQ;
        if (*freq_hz > SOUND_SINK_BASE) *freq_hz = SOUND_SINK_BASE;
        
        *period_ms = 500 + (int32_t)((climb_m_s - 0.5f) * 222.22f);
        if (*period_ms > 1500) *period_ms = 1500;
        if (*period_ms < 500) *period_ms = 500;
        
        *continuous = false;
    }
}


#if SX1262_ENABLED
// SX1262
//SX126x lora(LORA_NSS, LORA_RST, LORA_BUSY, LORA_DIO1,
//            LORA_SPI_SCK, LORA_SPI_MISO, LORA_SPI_MOSI);

static SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY); // NSS, DIO1, NRST, BUSY
static SX1262 g_radio = new Module(8, 14, 12, 13); // NSS, DIO1, NRST, BUSY

// ============================================================================
// Передача CW-тона через SX1262
// ============================================================================
void transmit_cw_tone(int32_t freq_hz, int32_t period_ms, bool continuous) {
    if (freq_hz == 0) {
        // Останавливаем передачу (если была активна)
        radio.standby();
        return;
    }
    
    // В режиме CW тон - это просто включение/выключение несущей с определенной частотой
    // Несущая всегда на CW_FREQUENCY, а тон - это модуляция включения/выключения
    static unsigned long last_toggle = 0;
    static bool carrier_on = false;
    
    if (continuous) {
        // Непрерывный тон - просто включаем несущую
        if (!carrier_on) {
            radio.setTxContinuousWave(CW_FREQUENCY, CW_POWER, 0);
            carrier_on = true;
        }
        last_toggle = millis();  // сбрасываем таймер для прерывистого режима
    } else if (period_ms > 0) {
        // Прерывистый тон
        unsigned long now = millis();
        if (now - last_toggle >= period_ms) {
            carrier_on = !carrier_on;
            last_toggle = now;
            
            if (carrier_on) {
                radio.setTxContinuousWave(CW_FREQUENCY, CW_POWER, 0);
            } else {
                radio.standby();
            }
        }
    }
}

// ============================================================================
// Передача CW тона через SX1262
// ============================================================================
void send_cw_tone(int32_t freq_hz, int32_t duration_ms) {
    if (freq_hz <= 0) {
        g_radio.sleep();
        return;
    }
    
    // Преобразуем частоту звука в период для CW
    // Для CW используем режим непрерывной несущей с модуляцией тоном
    // В RadioLib есть метод setTxContinuousWave для тестирования,
    // но для реальной передачи тона используем обычную передачу с прерываниями
    
    // Останавливаем предыдущую передачу
    g_radio.standby();
    
    // Настраиваем параметры для передачи
    int state = g_radio.startTransmitCW(LORA_FREQ, LORA_POWER);
    
    if (state == RADIOLIB_ERR_NONE) {
        // Имитируем тональный сигнал, изменяя состояние несущей
        // В реальном режиме CW несущая просто включена/выключена
        // Здесь мы используем startTransmit с короткими пакетами для имитации тона
        
        // Для настоящего CW нужна более сложная модуляция,
        // но в рамках данной реализации используем включение/выключение несущей
        delay(duration_ms);
        g_radio.sleep();
    } else {
        Serial.print("Transmit erroor CW: ");
        Serial.println(state);
    }
}
#endif

// ============================================================================
// Излучение тонального сигнала 
// ============================================================================
void transmit_tone(int32_t freq_hz, int32_t period_ms, bool continuous, int32_t duration_ms) {
    if (freq_hz <= 0) {
//        g_radio.sleep();
        return;
    }
    
    if (continuous) {
        // Непрерывный тон - включаем несущую на длительность
//        g_radio.startTransmitCW(LORA_FREQ, LORA_POWER);
        delay(duration_ms);
//        g_radio.sleep();
    } else {
        // Прерывистый тон - пульсирующая несущая
        int half_period = period_ms / 2;
        int repetitions = duration_ms / period_ms;
        if (repetitions < 1) repetitions = 1;
        
        for (int i = 0; i < repetitions; i++) {
//            g_radio.startTransmitCW(LORA_FREQ, LORA_POWER);
            delay(half_period);
//            g_radio.sleep();
            delay(half_period);
        }
    }
}

// ============================================================================
// симуляция данных сенсора для отладки
// эмуляция изменения вертикальной скорости
// ============================================================================
void simulate_sensor(void) 
#if SIMULATION_ENABLED
{
    int32_t pressure, temperature;
    int16_t climb;

    unsigned long now_debug = millis();
    if (now_debug - g_debug_last_toggle > 5000) {  // Меняем направление каждые 5 сек
        g_debug_last_toggle = now_debug;
        g_debug_climb_direction = !g_debug_climb_direction;
    }
    
    // Плавное изменение скорости от -3 до +3 м/с
    float progress = (now_debug % 10000) / 10000.0f;  // 0..1 за 10 сек
    float climb_m_s = (progress * 6.0f) - 3.0f;       // -3..+3 м/с
    climb = (int16_t)(climb_m_s * 100.0f);
    g_climb_filtered = (g_climb_filtered * 7 + climb) / 8;
    
    // Эмулируем изменение высоты
    static int32_t emulated_alt = 0;
    emulated_alt += g_climb_filtered * (dt / 1000);
    g_altitude_filtered = emulated_alt;
}
#else
{
}
#endif
// ============================================================================
// Обработка данных вариометра
// ============================================================================
void process_vario_data(void) {
    static unsigned long last_sample_time = 0;
    unsigned long now = millis();
    dt = now - last_sample_time;
    
    if (dt < SAMPLE_INTERVAL_MS) return;
    last_sample_time = now;
    
    int32_t pressure, temperature;
    int16_t climb;
    
    if (g_debug_mode) {
        simulate_sensor();
        } else {
        // РЕЖИМ РАБОТЫ С ДАТЧИКОМ
        if (!read_dps310(&pressure, &temperature)) {
            return;
        }
        
        if (g_altitude_base == 0) {
            g_altitude_base = pressure;
        }
        
        int32_t altitude = calculate_altitude(pressure, g_altitude_base);
        int32_t filtered_alt = kalman_update(&g_kalman, altitude);
        g_altitude_filtered = filtered_alt;
        
        int32_t delta_alt = filtered_alt - g_prev_altitude;
        float dt_sec = dt / 1000.0f;
        if (dt_sec > 0.1f) dt_sec = 0.1f;
        climb = (int16_t)(delta_alt / dt_sec);
        g_climb_filtered = (g_climb_filtered * 7 + climb) / 8;
        
        g_prev_altitude = filtered_alt;
    }
    
    // Расчет параметров звука
    int32_t freq, period;
    bool continuous;
    calculate_sound_params(g_climb_filtered, &freq, &period, &continuous);
    
    // Передача CW-тона
    #if SX1262_ENABLED
    transmit_cw_tone(freq, period, continuous);
    #endif
    transmit_tone(freq, period, continuous, 0.5);
    //void transmit_tone(int32_t freq_hz, int32_t period_ms, bool continuous, int32_t duration_ms) {
     
    // Индикация на LED
    static bool led_state = false;
    if (abs(g_climb_filtered) > 20) {
        if (millis() - last_sample_time > 100) {
            led_state = !led_state;
            digitalWrite(LED_BUILTIN, led_state);
        }
    } else {
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

// Declare a function pointer pointing to address 0 for MCU ATMEGA reset purpose

void (* resetFunc) (void) = 0; 

// ============================================================================
// Калибровка
// ============================================================================
void perform_calibration(void) {
    if (g_debug_mode) {
        DBG_PRINTLN("[DBG] Calibration omited");
        return;
    }
    
    int32_t sum_pressure = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < 20; i++) {
        int32_t pressure, temperature;
        if (read_dps310(&pressure, &temperature)) {
            sum_pressure += pressure;
            valid_readings++;
        }
        delay(20);
    }
    
    if (valid_readings > 0) {
        g_altitude_base = sum_pressure / valid_readings;
    }
    
    kalman_init(&g_kalman, KALMAN_Q, KALMAN_R, 0, KALMAN_INIT_P);
    g_prev_altitude = 0;
    g_climb_filtered = 0;
    g_altitude_filtered = 0;
    
    // Сигнал подтверждения калибровки (короткие бипы)
    for (int i = 0; i < 5; i++) {
        #if SX1262_ENABLED
        radio.setTxContinuousWave(CW_FREQUENCY, CW_POWER, 0);
        delay(100);
        radio.standby();
        delay(100);
        #else
//        tone(700,0.5);
        delay(200);
//        tone(700,0.5);
        delay(200);
        #endif
    }
    
    g_calibration_requested = false;
    DBG_PRINTLN("[DBG] Calibration passed");
}

// ============================================================================
// Обработчик кнопки калибровки
// ============================================================================
void handle_calibration_button() {
    static unsigned long last_button_time = 0;
    unsigned long now = millis();
    
    if (now - last_button_time < 200) return;
    last_button_time = now;
    
    if (digitalRead(CALIB_BUTTON_PIN) == LOW) {
        g_calibration_requested = true;
        DBG_PRINTLN("Calibration request by button");
    }
}

// ============================================================================
// Обработка последовательных команд
// ============================================================================
void handle_serial_commands() {
    if (!Serial.available()) return;
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "CAL" || cmd == "C") {
        g_calibration_requested = true;
        DBG_PRINTLN("Calibration command accepted");
    } else if (cmd == "DEBUG" || cmd == "D") {
        g_debug_mode = !g_debug_mode;
        DBG_PRINT("Debug mode: ");
        DBG_PRINT(g_debug_mode ? "ON" : "OFF");
        if (g_debug_mode) {
            DBG_PRINTLN("Vertical speed emulation -3..+3 m/s");
        }
    } else if (cmd == "STATUS" || cmd == "S") {
        DBG_PRINT("Heigh: ");
        DBG_PRINT(g_altitude_filtered / 100.0f, 1);
        DBG_PRINT(" m, Vertical speed: ");
        DBG_PRINTLN(g_climb_filtered / 100.0f, 1);
        DBG_PRINTLN(" m/s");
        DBG_PRINT("Base of pressure: ");
        DBG_PRINTLN(g_altitude_base / 10000.0f, 2);
    }
}

// ============================================================================
// Handle Serial Commands
// ============================================================================
void handle_serial_commands_ext() {
    if (!Serial.available()) return;
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd.length() == 0) return;
    
    if (cmd == "C") {
        g_calibration_requested = true;
        DBG_PRINTLN("[CMD] Calibration requested");
    }
    else if (cmd == "R") {
        DBG_PRINTLN("[CMD] Resetting device...");
        resetFunc(); // Call the function to jump to address 0
      //  ESP.restart();
    }
    else if (cmd.startsWith("F")) {
        // Format: F434075000
        String freqStr = cmd.substring(1);
        uint32_t freq = freqStr.toInt();
//        set_rf_frequency(freq);
    }
    else if (cmd == "?" || cmd == "help") {
        DBG_PRINTLN("\n=== Commands ===");
        DBG_PRINTLN("  C       - Calibrate sensor");
        DBG_PRINTLN("  R       - Restart device");
        #if  SX1262_ENABLED || RF_ENABLED
        DBG_PRINTLN("  F<freq> - Set RF frequency (Hz), e.g., F434075000");
        DBG_PRINTF("  Current RF: %d Hz (%.3f MHz)\n", rf_frequency, rf_frequency/1000000.0f);
        #endif
        DBG_PRINTLN("  ? / help- Show this help");
        DBG_PRINTLN("================\n");
    }
    else {
        DBG_PRINTF("[CMD] Unknown command: %s (type ? for help)\n", cmd.c_str());
    }
}

// ============================================================================
// Настройка
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    DBG_PRINTLN("\n=== Vario DPS310 sim ===");
    
    // Инициализация пинов
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
    
    // Инициализация I2C для DPS310
    Wire.begin();
//    Wire.begin(SDA_PIN, SCL_PIN);

    if (!dps.begin_I2C(0x76)) {
        DBG_PRINTLN("DPS310 baro sensor on I2C BUS not response!");
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
    }
    DBG_PRINTLN("Adafruit_DPS310 baro sensor initialized");
    
    // Настройка DPS310
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    
    // Инициализация SX1262
    #if SX1262_ENABLED || RF_ENABLED
    DBG_PRINTLN("Init SX1262...");
    int state = radio.begin(CW_FREQUENCY, 125000, 7, 4, 0x12, 22, 8, 1.6);
    if (state != RADIOLIB_ERR_NONE) {
        DBG_PRINT("Init Error SX1262: ");
        DBG_PRINTLN(state);
        while (1);
    }
    DBG_PRINTLN("SX1262 initialized");
    DBG_PRINT("Frequency: ");
    DBG_PRINT(CW_FREQUENCY / 1000000.0f, 3);
    DBG_PRINTLN(CW_POWER);
    #endif
    
    // Инициализация фильтра Калмана
    kalman_init(&g_kalman, KALMAN_Q, KALMAN_R, 0, KALMAN_INIT_P);
    
    // Первичная калибровка
    perform_calibration();
    
    DBG_PRINTLN("Commands: ?/help");
}

// ============================================================================
// Основной цикл
// ============================================================================
void loop() {
    // Обработка кнопки
    handle_calibration_button();
    
    // Обработка команд Serial
    handle_serial_commands_ext();
    
    // Обработка калибровки
    if (g_calibration_requested) {
        perform_calibration();
    }
    
    // Обработка данных вариометра
    process_vario_data();
    
    // Отладка в Serial
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 2000) {
        last_debug = millis();
        int32_t freq, period;
        bool continuous;
        calculate_sound_params(g_climb_filtered, &freq, &period, &continuous);
        #if VERBOSE_ENABLED
        DBG_PRINT("H: ");
        DBG_PRINT(g_altitude_filtered / 100.0f, 1);
        DBG_PRINT(" m, V: ");
        DBG_PRINT(g_climb_filtered / 100.0f, 1);
        DBG_PRINTLN(" m/s");
        if (freq > 0) {
            DBG_PRINT(", Tone: ");
            DBG_PRINT(freq);
            DBG_PRINT(" Hz");
            if (!continuous) {
                DBG_PRINT(", period ");
                DBG_PRINT(period);
                DBG_PRINTLN(" ms");
            }
        } else {
            DBG_PRINTLN(", silence");
        }
        DBG_PRINTLN();
        #endif
    }
    // Небольшая задержка для стабильности
    delay(10);
}

#if 0
//#############################################
const float CLIMB_FACTOR = 120.0; // Крутизна роста тона (Гц / (м/с)) при подъеме
const float DEADBAND = 0.1; // м/с
//#define SOUND_MIN_FREQ        200          // Гц
#define SOUND_MAX_FREQ        1200         // Гц

const float MIN_FREQ = SOUND_MIN_FREQ;     // 300 Нижняя граница слышимости
const float MAX_FREQ = SOUND_MAX_FREQ //3000.0;    // Верхняя граница для защиты динамика

void updateAudio(float vario_speed) {
  unsigned long now = millis();
  bool shouldBeep = false;
  int freq = SOUND_CLIMB_BASE;
  
  // Проверка на зону молчания (Deadband)
  if (abs(vario_speed) < DEADBAND) {
    noTone(BUZZER_PIN);
    return;
  }
  
  // Расчет частоты (тональности) в зависимости от подъема или спуска
  if (vario_speed >= 0) {
    // Подъем: частота растет линейно
    freq = SOUND_CLIMB_BASE + (vario_speed * CLIMB_FACTOR);
    freq = constrain(freq, (int)SOUND_CLIMB_BASE, (int)MAX_FREQ);
    shouldBeep = true; // Непрерывный сигнал
    isAlarmBeeping = false;
  } 
  else {
    // Спуск: частота падает линейно. Чем быстрее падаем, тем ниже тон.
    // Но если падение очень сильное (ниже SINK_ALARM_THRESH), включаем "панический" режим (прерывистый).
    float abs_sink = abs(vario_speed);
    freq = SOUND_CLIMB_BASE - (abs_sink * SINK_FACTOR);
    freq = constrain(freq, (int)MIN_FREQ, (int)SOUND_CLIMB_BASE);
    
    if (vario_speed < SINK_ALARM_THRESH) {
      // Режим быстрого снижения (прерывистый звук, как в IQ-ONE)
      if (now - lastBeepToggle >= BEEP_INTERVAL_MS) {
        beepState = !beepState;
        lastBeepToggle = now;
      }
      shouldBeep = beepState;
      isAlarmBeeping = true;
    } 
    else {
      // Обычный спуск (непрерывный низкий тон)
      shouldBeep = true;
      isAlarmBeeping = false;
    }
  }
  
  // Воспроизведение
  if (shouldBeep) {
    tone(PIN_BUZZER, freq);
  } else {
    noTone(PIN_BUZZER);
  }
}
//#####################################
#endif 

#if 0
Работа с SX1262 в режиме CW
Ключевая особенность реализации использование метода setTxContinuousWave() 
для генерации непрерывной несущей. Он создаёт чистый CW-сигнал, 
который можно принять на любой PMR-радиоприёмник, настроенный на частоту 433.075 МГц.

Передача тональной информации
Тон вариометра формируется путём прерывания несущей:
**Подъём** несущая включена постоянно, что даёт непрерывный тон
**Снижение** несущая включается/выключается с частотой, пропорциональной скорости снижения

Алгоритм звука повторяет логику Brauniger IQ ONE:
Подъём: непрерывный тон 600-1200 Гц
Снижение: прерывистый тон 200-600 Гц с периодом 500-1500 мс
Нейтральная зона: тишина (+-10 см/с)

Настройки для правильной работы

Перед загрузкой скетча** установите библиотеки через менеджер:
RadioLib для работы с SX1262
Adafruit DPS310 для датчика давления

Пины SX1262** настроены для Heltec LoRa V2:

#define LORA_CS   8
#define LORA_DIO1 14
#define LORA_RST  12
#define LORA_BUSY 13

Режим отладки

Для проверки работы без датчика отправьте в Serial команду DEBUG
Устройство начнёт эмулировать изменение вертикальной скорости от -3 до +3 м/с
#endif
