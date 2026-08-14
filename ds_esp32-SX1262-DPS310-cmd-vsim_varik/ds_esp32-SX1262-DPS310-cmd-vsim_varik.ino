/*****************************************************************************
 * Вариометр на Heltec LoRa V2 (ESP32-S3 + SX1262) + DPS310
 * Передача тональных сигналов через LoRa в режиме CW
 * Эмуляция звука Brauniger IQ ONE
 * Частота: 433.075 МГц
 *****************************************************************************/

#include <Wire.h>
#include <Dps310.h>
#include <RadioLib.h>

// ============================================================================
// Конфигурация
// ============================================================================
#define SAMPLE_INTERVAL_MS    50      // 20 Гц
#define CALIB_BUTTON_PIN      0       // Кнопка калибровки (GPIO0 - BOOT)

#define LORA_FREQ             433.075 // Частота передачи в МГц
#define LORA_POWER            22      // Мощность в dBm (макс 22)
#define LORA_SPREADING_FACTOR 12
#define LORA_BANDWIDTH        125.0
#define LORA_CODING_RATE      8

#define CW_TONE_DURATION_MS   100     // Длительность тона в мс
#define CW_SILENCE_DURATION_MS 50     // Пауза между тонами

#define SOUND_NEUTRAL_ZONE    10      // см/с - нейтральная зона
#define SOUND_MIN_FREQ        200     // Гц
#define SOUND_MAX_FREQ        1200    // Гц

#define SIMULATION_ENABLED    true    // Режим эмуляции для отладки
#define SIMULATION_PERIOD_MS  1000    // Период изменения скорости при эмуляции

// ============================================================================
// Структуры данных
// ============================================================================
typedef struct {
    int32_t pressure;      // Давление в Па (x100)
    int32_t temperature;   // Температура в °C (x100)
    int32_t altitude;      // Высота в см
    int16_t climb;         // Вертикальная скорость в см/с
} vario_data_t;

typedef struct {
    int32_t x;             // Состояние (высота) в см
    int32_t p;             // Ковариация (x100)
    int32_t q;             // Шум процесса (x100)
    int32_t r;             // Шум измерения (x100)
    int32_t k;             // Коэффициент Калмана (x100)
} kalman_filter_t;

// ============================================================================
// Глобальные переменные
// ============================================================================
static vario_data_t g_vario_data;
static kalman_filter_t g_kalman;
static int32_t g_altitude_base = 0;
static int16_t g_climb_filtered = 0;
static int32_t g_altitude_filtered = 0;
static uint32_t g_last_sample_time = 0;
static int32_t g_prev_altitude = 0;
static bool g_calibration_requested = false;

// ДПС310
static Dps310 g_dps310 = Dps310();

// SX1262
static SX1262 g_radio = new Module(8, 14, 12, 13); // NSS, DIO1, NRST, BUSY

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
    int32_t denominator = k->p + k->r;
    if (denominator != 0) {
        k->k = (k->p * 100) / denominator;
    }
    
    int32_t error = measurement - k->x;
    k->x = k->x + (error * k->k) / 100;
    k->p = (k->p * (100 - k->k)) / 100;
    
    return k->x;
}

// ============================================================================
// ДПС310
// ============================================================================
bool read_sensor(int32_t *pressure, int32_t *temperature) {
    int32_t raw_pressure, raw_temperature;
    
    if (g_dps310.measureBoth() != 0) {
        return false;
    }
    
    // Ждем завершения измерения
    delay(100);
    
    if (g_dps310.getPressureAndTemperature(raw_pressure, raw_temperature) != 0) {
        return false;
    }
    
    // Давление: raw_pressure в Па * 100 (для целочисленной арифметики)
    *pressure = raw_pressure;
    *temperature = raw_temperature; // °C * 100
    
    return true;
}

int32_t calculate_altitude(int32_t pressure, int32_t base_pressure) {
    if (pressure <= 0 || base_pressure <= 0) {
        return 0;
    }
    
    // Упрощенное приближение: ~8.5 м на 1 гПа
    int32_t diff = base_pressure - pressure;
    return (diff * 85) / 10;
}

// ============================================================================
// Генерация звуковых параметров (Brauniger IQ ONE)
// ============================================================================
void calculate_sound_parameters(int16_t climb_cm_s, int32_t *freq_hz, int32_t *period_ms, bool *continuous) {
    int16_t abs_climb = (climb_cm_s > 0) ? climb_cm_s : -climb_cm_s;
    
    // Нейтральная зона
    if (abs_climb < SOUND_NEUTRAL_ZONE) {
        *freq_hz = 0;
        *period_ms = 0;
        *continuous = false;
        return;
    }
    
    if (abs_climb > 500) abs_climb = 500;
    
    if (climb_cm_s > 0) {
        // ПОДЪЕМ: непрерывный тон, частота 600-1200 Гц
        float climb_m_s = abs_climb / 100.0f;
        if (climb_m_s < 0.5f) climb_m_s = 0.5f;
        if (climb_m_s > 5.0f) climb_m_s = 5.0f;
        
        *freq_hz = 600 + (int32_t)((climb_m_s - 0.5f) * 133.33f);
        if (*freq_hz > SOUND_MAX_FREQ) *freq_hz = SOUND_MAX_FREQ;
        if (*freq_hz < 600) *freq_hz = 600;
        
        *period_ms = 0;
        *continuous = true;
    } else {
        // СНИЖЕНИЕ: прерывистый тон, частота 200-600 Гц
        float climb_m_s = abs_climb / 100.0f;
        if (climb_m_s < 0.5f) climb_m_s = 0.5f;
        if (climb_m_s > 5.0f) climb_m_s = 5.0f;
        
        *freq_hz = 600 - (int32_t)((climb_m_s - 0.5f) * 88.88f);
        if (*freq_hz < SOUND_MIN_FREQ) *freq_hz = SOUND_MIN_FREQ;
        if (*freq_hz > 600) *freq_hz = 600;
        
        *period_ms = 500 + (int32_t)((climb_m_s - 0.5f) * 222.22f);
        if (*period_ms > 1500) *period_ms = 1500;
        if (*period_ms < 500) *period_ms = 500;
        
        *continuous = false;
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
        Serial.print("Ошибка передачи CW: ");
        Serial.println(state);
    }
}

// ============================================================================
// Передача тонального сигнала (эмуляция CW через включение/выключение)
// ============================================================================
void transmit_tone(int32_t freq_hz, int32_t period_ms, bool continuous, int32_t duration_ms) {
    if (freq_hz <= 0) {
        g_radio.sleep();
        return;
    }
    
    if (continuous) {
        // Непрерывный тон - включаем несущую на длительность
        g_radio.startTransmitCW(LORA_FREQ, LORA_POWER);
        delay(duration_ms);
        g_radio.sleep();
    } else {
        // Прерывистый тон - пульсирующая несущая
        int half_period = period_ms / 2;
        int repetitions = duration_ms / period_ms;
        if (repetitions < 1) repetitions = 1;
        
        for (int i = 0; i < repetitions; i++) {
            g_radio.startTransmitCW(LORA_FREQ, LORA_POWER);
            delay(half_period);
            g_radio.sleep();
            delay(half_period);
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
    
    // Эмуляция для отладки
    #if SIMULATION_ENABLED
    static unsigned long last_sim_change = 0;
    static int16_t sim_climb = 0;
    static int32_t sim_altitude = 0;
    
    if (now - last_sim_change > SIMULATION_PERIOD_MS) {
        last_sim_change = now;
        // Периодически меняем скорость: +2, +1, 0, -1, -2 м/с
        static int sim_index = 0;
        int sim_values[] = {200, 150, 80, 0, -80, -150, -200, -100, 0, 100};
        sim_climb = sim_values[sim_index % (sizeof(sim_values)/sizeof(sim_values[0]))];
        sim_index++;
        
        // Меняем высоту в соответствии со скоростью
        sim_altitude += sim_climb * (SIMULATION_PERIOD_MS / 1000);
        if (sim_altitude < 0) sim_altitude = 0;
        
        Serial.print("Симуляция: V=");
        Serial.print(sim_climb / 100.0f);
        Serial.print(" м/с, H=");
        Serial.print(sim_altitude / 100.0f);
        Serial.println(" м");
    }
    
    g_altitude_filtered = sim_altitude;
    g_climb_filtered = sim_climb;
    g_vario_data.climb = sim_climb;
    g_vario_data.altitude = sim_altitude;
    
    #else
    // Реальный режим - чтение сенсора
    int32_t pressure, temperature;
    if (!read_sensor(&pressure, &temperature)) {
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
    int16_t climb = (int16_t)(delta_alt / dt_sec);
    
    g_climb_filtered = (g_climb_filtered * 7 + climb) / 8;
    g_prev_altitude = filtered_alt;
    
    g_vario_data.pressure = pressure;
    g_vario_data.temperature = temperature;
    g_vario_data.altitude = filtered_alt;
    g_vario_data.climb = g_climb_filtered;
    #endif
    
    // Расчет звуковых параметров
    int32_t freq, period;
    bool continuous;
    calculate_sound_parameters(g_climb_filtered, &freq, &period, &continuous);
    
    // Передача тона по радио
    static unsigned long last_tone_time = 0;
    unsigned long tone_duration = continuous ? CW_TONE_DURATION_MS : CW_TONE_DURATION_MS + CW_SILENCE_DURATION_MS;
    
    if (freq > 0 && (now - last_tone_time >= tone_duration)) {
        last_tone_time = now;
        transmit_tone(freq, period, continuous, CW_TONE_DURATION_MS);
    }
    
    // Отладка
    static unsigned long last_debug = 0;
    if (now - last_debug > 2000) {
        last_debug = now;
        Serial.print("H: ");
        Serial.print(g_vario_data.altitude / 100.0f, 1);
        Serial.print(" м, V: ");
        Serial.print(g_vario_data.climb / 100.0f, 1);
        Serial.print(" м/с, Тон: ");
        if (freq > 0) {
            Serial.print(freq);
            Serial.print(" Гц");
            if (!continuous) {
                Serial.print(", период ");
                Serial.print(period);
                Serial.print(" мс");
            }
        } else {
            Serial.print("тишина");
        }
        Serial.println();
    }
}

// ============================================================================
// Калибровка
// ============================================================================
void perform_calibration(void) {
    #if !SIMULATION_ENABLED
    int32_t sum_pressure = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < 20; i++) {
        int32_t pressure, temperature;
        if (read_sensor(&pressure, &temperature)) {
            sum_pressure += pressure;
            valid_readings++;
        }
        delay(20);
    }
    
    if (valid_readings > 0) {
        g_altitude_base = sum_pressure / valid_readings;
    }
    #endif
    
    kalman_init(&g_kalman, 10, 50, 0, 100);
    g_prev_altitude = 0;
    g_climb_filtered = 0;
    g_altitude_filtered = 0;
    g_calibration_requested = false;
    
    Serial.println("Калибровка выполнена");
}

// ============================================================================
// Инициализация
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Вариометр Heltec LoRa V2 ===");
    
    // Кнопка калибровки
    pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
    
    // Инициализация I2C для DPS310
    Wire.begin();
    
    // Инициализация DPS310
    if (g_dps310.begin() != 0) {
        Serial.println("Ошибка инициализации DPS310!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("DPS310 инициализирован");
    
    // Настройка DPS310
    g_dps310.setTempScale(DPS310_TEMP_SCALE_2);
    g_dps310.setPressScale(DPS310_PRESS_SCALE_2);
    g_dps310.setTempRate(DPS310_TEMP_RATE_4);
    g_dps310.setPressRate(DPS310_PRESS_RATE_4);
    g_dps310.setMode(DPS310_MODE_CONT);
    
    // Инициализация SX1262
    Serial.print("Инициализация SX1262... ");
    int state = g_radio.begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODING_RATE, 0x12, LORA_POWER, 8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Ошибка: ");
        Serial.println(state);
        while (1) {
            delay(1000);
        }
    }
    Serial.println("OK");
    
    // Устанавливаем частоту в режиме CW
    g_radio.setFrequency(LORA_FREQ);
    
    // Инициализация фильтра Калмана
    kalman_init(&g_kalman, 10, 50, 0, 100);
    
    // Первичная калибровка
    perform_calibration();
    Serial.println("Система готова к работе");
}

// ============================================================================
// Основной цикл
// ============================================================================
void loop() {
    // Обработка кнопки калибровки
    if (digitalRead(CALIB_BUTTON_PIN) == LOW) {
        delay(50); // Антидребезг
        if (digitalRead(CALIB_BUTTON_PIN) == LOW) {
            g_calibration_requested = true;
            while (digitalRead(CALIB_BUTTON_PIN) == LOW) {
                delay(10);
            }
        }
    }
    
    if (g_calibration_requested) {
        perform_calibration();
    }
    
    // Обработка данных
    process_vario_data();
    
    // Небольшая задержка
    delay(10);
}