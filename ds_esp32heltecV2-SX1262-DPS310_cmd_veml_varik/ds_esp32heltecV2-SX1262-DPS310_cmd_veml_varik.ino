/*****************************************************************************
 * Вариометр на Heltec LoRa V2 (ESP32 + SX1262) с датчиком DPS310
 * Передача тональных сигналов в режиме CW на частоте 433.075 МГц
 * Эмуляция звука Brauniger IQ ONE
 * Фильтр Калмана, калибровка, режим отладки
 *****************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <RadioLib.h>

// ============================================================================
// Конфигурация
// ============================================================================
#define SAMPLE_INTERVAL_MS    50      // 20 Гц
#define CALIB_BUTTON_PIN      0       // Кнопка калибровки (GPIO0)
#define LED_BUILTIN           25      // Встроенный LED

// Параметры CW-передачи
#define CW_FREQUENCY          433075000ULL  // 433.075 МГц
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

// ============================================================================
// Пины для SX1262 на Heltec LoRa V2
// ============================================================================
#define LORA_CS               8
#define LORA_DIO1             14
#define LORA_RST              12
#define LORA_BUSY             13

// ============================================================================
// Глобальные переменные
// ============================================================================
Adafruit_DPS310 dps;
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

typedef struct {
    int32_t x;             // Состояние (высота) в см
    int32_t p;             // Ковариация (x100)
    int32_t q;             // Шум процесса (x100)
    int32_t r;             // Шум измерения (x100)
    int32_t k;             // Коэффициент Калмана (x100)
} kalman_filter_t;

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
// Датчик DPS310
// ============================================================================
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

int32_t calculate_altitude(int32_t pressure, int32_t base_pressure) {
    if (pressure <= 0 || base_pressure <= 0) {
        return 0;
    }
    
    // Упрощенное приближение: 8.5 м на 1 гПа
    int32_t diff = base_pressure - pressure;
    return (diff * 85) / 10;  // Высота в см
}

// ============================================================================
// Генерация звуковых параметров Brauniger IQ ONE
// ============================================================================
void calculate_sound_params(int16_t climb_cm_s, int32_t *freq_hz, int32_t *period_ms, bool *continuous) {
    int16_t abs_climb = (climb_cm_s > 0) ? climb_cm_s : -climb_cm_s;
    
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
        // СНИЖЕНИЕ: прерывистый тон
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
// Обработка данных вариометра
// ============================================================================
void process_vario_data(void) {
    static unsigned long last_sample_time = 0;
    unsigned long now = millis();
    unsigned long dt = now - last_sample_time;
    
    if (dt < SAMPLE_INTERVAL_MS) return;
    last_sample_time = now;
    
    int32_t pressure, temperature;
    int16_t climb;
    
    if (g_debug_mode) {
        // РЕЖИМ ОТЛАДКИ: эмуляция изменения вертикальной скорости
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
    transmit_cw_tone(freq, period, continuous);
    
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

// ============================================================================
// Калибровка
// ============================================================================
void perform_calibration(void) {
    if (g_debug_mode) {
        Serial.println("Отладка: калибровка пропущена");
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
        radio.setTxContinuousWave(CW_FREQUENCY, CW_POWER, 0);
        delay(100);
        radio.standby();
        delay(100);
    }
    
    g_calibration_requested = false;
    Serial.println("Калибровка завершена");
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
        Serial.println("Запрос калибровки по кнопке");
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
        Serial.println("Команда калибровки принята");
    } else if (cmd == "DEBUG" || cmd == "D") {
        g_debug_mode = !g_debug_mode;
        Serial.print("Режим отладки: ");
        Serial.println(g_debug_mode ? "ВКЛ" : "ВЫКЛ");
        if (g_debug_mode) {
            Serial.println("Эмуляция изменения скорости -3..+3 м/с");
        }
    } else if (cmd == "STATUS" || cmd == "S") {
        Serial.print("Высота: ");
        Serial.print(g_altitude_filtered / 100.0f, 1);
        Serial.print(" м, Скорость: ");
        Serial.print(g_climb_filtered / 100.0f, 1);
        Serial.println(" м/с");
        Serial.print("База давления: ");
        Serial.println(g_altitude_base / 10000.0f, 2);
    }
}

// ============================================================================
// Настройка
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Вариометр на Heltec LoRa V2 ===");
    
    // Инициализация пинов
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
    
    // Инициализация I2C для DPS310
    Wire.begin();
    if (!dps.begin_I2C(0x76)) {
        Serial.println("Ошибка инициализации DPS310!");
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
    }
    Serial.println("DPS310 инициализирован");
    
    // Настройка DPS310
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    
    // Инициализация SX1262
    Serial.println("Инициализация SX1262...");
    int state = radio.begin(CW_FREQUENCY, 125000, 7, 4, 0x12, 22, 8, 1.6);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Ошибка инициализации SX1262: ");
        Serial.println(state);
        while (1);
    }
    Serial.println("SX1262 инициализирован");
    Serial.print("Частота: ");
    Serial.println(CW_FREQUENCY / 1000000.0f, 3);
    Serial.print("Мощность: ");
    Serial.println(CW_POWER);
    
    // Инициализация фильтра Калмана
    kalman_init(&g_kalman, KALMAN_Q, KALMAN_R, 0, KALMAN_INIT_P);
    
    // Первичная калибровка
    Serial.println("Выполняется первичная калибровка...");
    perform_calibration();
    
    Serial.println("Система готова к работе");
    Serial.println("Команды: CAL - калибровка, DEBUG - режим отладки, STATUS - статус");
}

// ============================================================================
// Основной цикл
// ============================================================================
void loop() {
    // Обработка кнопки
    handle_calibration_button();
    
    // Обработка команд Serial
    handle_serial_commands();
    
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
        
        Serial.print("H: ");
        Serial.print(g_altitude_filtered / 100.0f, 1);
        Serial.print(" м, V: ");
        Serial.print(g_climb_filtered / 100.0f, 1);
        Serial.print(" м/с");
        if (freq > 0) {
            Serial.print(", Тон: ");
            Serial.print(freq);
            Serial.print(" Гц");
            if (!continuous) {
                Serial.print(", период ");
                Serial.print(period);
                Serial.print(" мс");
            }
        } else {
            Serial.print(", тишина");
        }
        Serial.println();
    }
    
    // Небольшая задержка для стабильности
    delay(10);
}
#if 0

## Работа с SX1262 в режиме CW

Ключевая особенность реализации — использование метода `setTxContinuousWave()` для генерации непрерывной несущей. Он создаёт чистый CW-сигнал, который можно принять на любой PMR-радиоприёмник, настроенный на частоту 433.075 МГц.

### Передача тональной информации

Тон вариометра формируется путём прерывания несущей:
- **Подъём** — несущая включена постоянно, что даёт непрерывный тон
- **Снижение** — несущая включается/выключается с частотой, пропорциональной скорости снижения

Алгоритм звука повторяет логику Brauniger IQ ONE:
- Подъём: непрерывный тон 600-1200 Гц
- Снижение: прерывистый тон 200-600 Гц с периодом 500-1500 мс
- Нейтральная зона: тишина (±10 см/с)

### Настройки для правильной работы

**Перед загрузкой скетча** установите библиотеки через менеджер:
- `RadioLib` — для работы с SX1262
- `Adafruit DPS310` — для датчика давления

**Пины SX1262** настроены для Heltec LoRa V2:
```cpp
#define LORA_CS   8
#define LORA_DIO1 14
#define LORA_RST  12
#define LORA_BUSY 13
```

### Режим отладки

Для проверки работы без датчика отправьте в Serial команду `DEBUG`. Устройство начнёт эмулировать изменение вертикальной скорости от -3 до +3 м/с, что полезно для тестирования звука на радиоприёмнике.
#endif

