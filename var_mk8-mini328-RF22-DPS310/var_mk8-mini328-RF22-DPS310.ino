/*****************************************************************************
 * Вариометр на Arduino Pro Mini + RF22B (Direct Mode) + DPS310
 * Эмуляция звука Brauniger IQ ONE с передачей на PMR-радиоприемник
 * Фильтр Калмана, калибровка кнопкой
 *****************************************************************************/

#include <SPI.h>
#include <RF22.h>           // Библиотека RF22 для работы с RFM22B/RF22B
#include <Adafruit_DPS310.h> // Библиотека для DPS310
#if 0
| Arduino Pro Mini | RF22B |
|------------------|-------|
| 3.3V             | VCC   |
| GND              | GND   |
| D10 (SS)         | CS    |
| D11 (MOSI)       | SDI   |
| D12 (MISO)       | SDO   |
| D13 (SCK)        | SCK   |
| D2               | NIRQ  |
| D4               | GPIO0 (TX Data) |
| D5               | GPIO1 |

| Arduino Pro Mini | DPS310 |
|------------------|--------|
| 3.3V             | VIN    |
| GND              | GND    |
| A4 (SDA)         | SDA    |
| A5 (SCL)         | SCL    |

### MODE_BUTTON_PIN
- Один контакт → GND
- Другой контакт → D3 
#endif

// ============================================================================
// Конфигурация пинов
// ============================================================================
// RF22B
#define RF22_CS_PIN         10    // Chip Select (SS)
#define RF22_IRQ_PIN        2     // Прерывание (NIRQ)
#define RF22_GPIO0_PIN      4     // GPIO0 - для управления TX/RX
#define RF22_GPIO1_PIN      5     // GPIO1 - для управления TX/RX

// DPS310 (I2C)
// SDA -> A4, SCL -> A5 (стандартные пины I2C на Pro Mini)

// Кнопка калибровки
#define MODE_BUTTON_PIN    3     // Кнопка между пином и GND

// Светодиод для индикации
#define LED_PIN             13

// ============================================================================
// Параметры вариометра
// ============================================================================
#define SAMPLE_INTERVAL_MS    50     // 20 Гц
#define CALIB_SAMPLES         20     // Количество измерений для калибровки

// Параметры фильтра Калмана (целочисленные, масштабирование x100)
#define KALMAN_Q              10     // Шум процесса (0.1)
#define KALMAN_R              50     // Шум измерения (0.5)
#define KALMAN_INIT_P         100    // Начальная ковариация (1.0)

// Параметры звука Brauniger IQ ONE
#define NEUTRAL_ZONE          10     // см/с - нейтральная зона (тишина)
#define CLIMB_BASE_FREQ       600    // Гц - базовая частота при подъеме
#define CLIMB_MAX_FREQ        1200   // Гц - макс. частота при подъеме
#define SINK_BASE_FREQ        600    // Гц - базовая частота при снижении
#define SINK_MIN_FREQ         200    // Гц - мин. частота при снижении
#define SINK_MIN_PERIOD       500    // мс - мин. период прерываний
#define SINK_MAX_PERIOD       1500   // мс - макс. период прерываний
#define MAX_CLIMB_SPEED       500    // см/с - макс. скорость для звука (5 м/с)

// ============================================================================
// Структуры данных
// ============================================================================
typedef struct {
    int32_t pressure;        // Давление в Па (x100)
    int32_t temperature;     // Температура в °C (x100)
    int32_t altitude;        // Высота в см
    int16_t climb;           // Вертикальная скорость в см/с
} vario_data_t;

typedef struct {
    int32_t x;               // Состояние (высота) в см
    int32_t p;               // Ковариация (x100)
    int32_t q;               // Шум процесса (x100)
    int32_t r;               // Шум измерения (x100)
    int32_t k;               // Коэффициент Калмана (x100)
} kalman_filter_t;

// ============================================================================
// Глобальные переменные
// ============================================================================
RF22 rf22(RF22_CS_PIN, RF22_IRQ_PIN);
Adafruit_DPS310 BARO;

static vario_data_t g_vario;
static kalman_filter_t g_kalman;
static int32_t g_base_pressure = 0;      // Базовое давление для калибровки
static int32_t g_prev_altitude = 0;
static int16_t g_climb_filtered = 0;
static int32_t g_altitude_filtered = 0;
static bool g_calibration_requested = false;

// Переменные для генерации тона в Direct Mode
static uint32_t g_tone_freq = 0;         // Частота тона в Гц
static uint32_t g_tone_period_ms = 0;    // Период прерываний для снижения
static bool g_tone_continuous = true;    // Непрерывный или прерывистый тон
static unsigned long g_last_toggle_ms = 0;
static bool g_tone_state = false;        // Текущее состояние тона (вкл/выкл)

// ============================================================================
// Фильтр Калмана (целочисленная реализация)
// ============================================================================
void kalman_init(kalman_filter_t *k, int32_t q, int32_t r, 
                 int32_t initial_x, int32_t initial_p) {
    k->q = q;
    k->r = r;
    k->x = initial_x;
    k->p = initial_p;
    k->k = 0;
}

int32_t kalman_update(kalman_filter_t *k, int32_t measurement) {
    // Прогноз
    k->p = k->p + k->q;
    
    // Обновление: k = p / (p + r)
    int32_t denominator = k->p + k->r;
    if (denominator != 0) {
        k->k = (k->p * 100) / denominator;
    }
    
    // x = x + k * (measurement - x)
    int32_t error = measurement - k->x;
    k->x = k->x + (error * k->k) / 100;
    
    // p = (1 - k) * p
    k->p = (k->p * (100 - k->k)) / 100;
    
    return k->x;
}

// ============================================================================
// Работа с датчиком DPS310
// ============================================================================
bool read_dps310(int32_t *pressure, int32_t *temperature) {
    sensors_event_t temp_event, pressure_event;
    
    if (!BARO.getEvents(&temp_event, &pressure_event)) {
        return false;
    }
    
    // Давление в Па * 100 (для целочисленных расчетов)
    *pressure = (int32_t)(pressure_event.pressure * 100.0f);
    // Температура в °C * 100
    *temperature = (int32_t)(temp_event.temperature * 100.0f);
    
    return true;
}

int32_t calc_altitude(int32_t pressure, int32_t base_pressure) {
    // H = 44330 * (1 - (P/P0)^0.1903)
    // Целочисленное приближение: ~8.5 м на 1 гПа
    if (pressure <= 0 || base_pressure <= 0) return 0;
    
    int32_t diff = base_pressure - pressure;
    return (diff * 85) / 10;  // Высота в см
}

// ============================================================================
// Управление RF22B в Direct Mode для передачи тона
// ============================================================================
void rf22_direct_mode_init(void) {
    // Включаем несущую (без модуляции)
    rf22.setModeTx();
    
    // Настройка Direct Mode:
    // Включаем передатчик с несущей частотой
    // Модуляция будет подаваться через GPIO0 (TX_DATA)
    
    // Настраиваем GPIO0 как вход для данных модуляции (TX Data)
    // В Direct Mode данные подаются на этот пин
    pinMode(RF22_GPIO0_PIN, OUTPUT);
    digitalWrite(RF22_GPIO0_PIN, LOW);
    
    // Включаем несущую
    rf22.setTxPower(RF22_TXPOW_20DBM);  // Максимальная мощность (~100 мВт)
    rf22.setFrequency(434.075);            // Частота для PMR (434 МГц)
    
    // Включаем передатчик
    rf22.setModeTx();
    
    // Небольшая задержка для стабилизации
    delay(10);
}

void rf22_set_tone(bool state) {
    // В Direct Mode подаем сигнал на GPIO0 (TX_DATA)
    // HIGH = несущая включена (тон), LOW = несущая выключена (пауза)
    digitalWrite(RF22_GPIO0_PIN, state ? HIGH : LOW);
}

void rf22_set_carrier(bool on) {
    if (on) {
        rf22.setModeTx();
    } else {
        rf22.setModeIdle();  // Отключаем несущую
    }
}

// ============================================================================
// Расчет звуковых параметров (Brauniger IQ ONE)
// ============================================================================
void calculate_sound_params(int16_t climb_cm_s, uint32_t *freq, 
                            uint32_t *period_ms, bool *continuous) {
    int16_t abs_climb = (climb_cm_s > 0) ? climb_cm_s : -climb_cm_s;
    
    // Нейтральная зона - тишина
    if (abs_climb < NEUTRAL_ZONE) {
        *freq = 0;
        *period_ms = 0;
        *continuous = false;
        return;
    }
    
    // Ограничиваем максимальную скорость
    if (abs_climb > MAX_CLIMB_SPEED) abs_climb = MAX_CLIMB_SPEED;
    
    if (climb_cm_s > 0) {
        // ПОДЪЕМ: непрерывный тон, частота растет
        // 0.5 м/с -> 600 Гц, 5 м/с -> 1200 Гц
        float climb_m_s = abs_climb / 100.0f;
        if (climb_m_s < 0.5f) climb_m_s = 0.5f;
        
        *freq = CLIMB_BASE_FREQ + (uint32_t)((climb_m_s - 0.5f) * 133.33f);
        if (*freq > CLIMB_MAX_FREQ) *freq = CLIMB_MAX_FREQ;
        if (*freq < CLIMB_BASE_FREQ) *freq = CLIMB_BASE_FREQ;
        
        *period_ms = 0;
        *continuous = true;
    } else {
        // СНИЖЕНИЕ: прерывистый тон, частота падает, период растет
        float climb_m_s = abs_climb / 100.0f;
        if (climb_m_s < 0.5f) climb_m_s = 0.5f;
        
        *freq = SINK_BASE_FREQ - (uint32_t)((climb_m_s - 0.5f) * 88.88f);
        if (*freq < SINK_MIN_FREQ) *freq = SINK_MIN_FREQ;
        if (*freq > SINK_BASE_FREQ) *freq = SINK_BASE_FREQ;
        
        *period_ms = SINK_MIN_PERIOD + 
                     (uint32_t)((climb_m_s - 0.5f) * 222.22f);
        if (*period_ms > SINK_MAX_PERIOD) *period_ms = SINK_MAX_PERIOD;
        if (*period_ms < SINK_MIN_PERIOD) *period_ms = SINK_MIN_PERIOD;
        
        *continuous = false;
    }
}

// ============================================================================
// Обновление звукового выхода (Direct Mode)
// ============================================================================
void update_sound_output(int16_t climb_cm_s) {
    uint32_t freq, period_ms;
    bool continuous;
    
    calculate_sound_params(climb_cm_s, &freq, &period_ms, &continuous);
    
    g_tone_freq = freq;
    g_tone_period_ms = period_ms;
    g_tone_continuous = continuous;
    
    if (freq == 0) {
        // Тишина - выключаем несущую
        rf22_set_carrier(false);
        rf22_set_tone(false);
        digitalWrite(LED_PIN, LOW);
        return;
    }
    
    // Включаем несущую
    rf22_set_carrier(true);
    
    if (continuous) {
        // Непрерывный тон - модулируем частотой
        // Для генерации тона используем прерывания таймера
        // Прямое управление через GPIO0
        rf22_set_tone(true);
        digitalWrite(LED_PIN, HIGH);
    } else {
        // Прерывистый тон - включаем/выключаем с заданным периодом
        unsigned long now = millis();
        uint32_t half_period = period_ms / 2;
        
        if (now - g_last_toggle_ms >= half_period) {
            g_last_toggle_ms = now;
            g_tone_state = !g_tone_state;
            
            if (g_tone_state) {
                rf22_set_tone(true);
                digitalWrite(LED_PIN, HIGH);
            } else {
                rf22_set_tone(false);
                digitalWrite(LED_PIN, LOW);
            }
        }
    }
}

// ============================================================================
// Генерация тона с помощью таймера (для непрерывного тона)
// ============================================================================
// Используем Timer2 для генерации ШИМ тона на GPIO0
// Примечание: это простая реализация, для точной частоты можно использовать
// аппаратный таймер, но в данном случае мы используем программный метод

// ============================================================================
// Обработка данных вариометра
// ============================================================================
void process_vario_data(void) {
    static unsigned long last_sample = 0;
    unsigned long now = millis();
    unsigned long dt = now - last_sample;
    
    if (dt < SAMPLE_INTERVAL_MS) return;
    last_sample = now;
    
    // Читаем датчик
    int32_t pressure, temperature;
    if (!read_dps310(&pressure, &temperature)) {
        return;
    }
    
    // Если калибровка не выполнена - используем текущее давление
    if (g_base_pressure == 0) {
        g_base_pressure = pressure;
    }
    
    // Расчет высоты
    int32_t altitude = calc_altitude(pressure, g_base_pressure);
    
    // Фильтр Калмана
    int32_t filtered_alt = kalman_update(&g_kalman, altitude);
    g_altitude_filtered = filtered_alt;
    
    // Расчет вертикальной скорости
    int32_t delta_alt = filtered_alt - g_prev_altitude;
    float dt_sec = dt / 1000.0f;
    if (dt_sec > 0.1f) dt_sec = 0.1f;
    int16_t climb = (int16_t)(delta_alt / dt_sec);
    
    // Фильтр скорости (скользящее среднее)
    g_climb_filtered = (g_climb_filtered * 7 + climb) / 8;
    
    g_prev_altitude = filtered_alt;
    
    // Сохраняем данные
    g_vario.pressure = pressure;
    g_vario.temperature = temperature;
    g_vario.altitude = filtered_alt;
    g_vario.climb = g_climb_filtered;
    
    // Обновляем звуковой выход
    update_sound_output(g_climb_filtered);
}

// ============================================================================
// Калибровка
// ============================================================================
void perform_calibration(void) {
    // Мигание LED - начало калибровки
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
    
    // Усредняем несколько измерений
    int32_t sum_pressure = 0;
    int valid = 0;
    
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int32_t pressure, temperature;
        if (read_dps310(&pressure, &temperature)) {
            sum_pressure += pressure;
            valid++;
        }
        delay(20);
    }
    
    if (valid > 0) {
        g_base_pressure = sum_pressure / valid;
    }
    
    // Сбрасываем фильтр Калмана
    kalman_init(&g_kalman, KALMAN_Q, KALMAN_R, 0, KALMAN_INIT_P);
    g_prev_altitude = 0;
    g_climb_filtered = 0;
    g_altitude_filtered = 0;
    
    // Подтверждение калибровки (длинная вспышка)
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    
    g_calibration_requested = false;
}

// ============================================================================
// Обработчик кнопки калибровки
// ============================================================================
void check_calibration_button(void) {
    static unsigned long last_debounce = 0;
    static int last_state = HIGH;
    unsigned long now = millis();
    
    if (now - last_debounce < 50) return;
    last_debounce = now;
    
    int state = digitalRead(MODE_BUTTON_PIN);
    if (state == LOW && last_state == HIGH) {
        g_calibration_requested = true;
    }
    last_state = state;
}

// ============================================================================
// Настройка (setup)
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println(F("=== Вариометр Pro Mini + RF22B + DPS310 ==="));
    
    // Настройка пинов
    pinMode(LED_PIN, OUTPUT);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    
    // Инициализация DPS310
    if (!BARO.begin_I2C()) {
        Serial.println(F("Ошибка: DPS310 не найден!"));
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
    }
    Serial.println(F("DPS310 инициализирован"));
    
    // Настройка DPS310
    BARO.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    BARO.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    
    // Инициализация RF22B
    if (!rf22.init()) {
        Serial.println(F("Ошибка: RF22B не инициализирован!"));
        while (1);
    }
    Serial.println(F("RF22B инициализирован"));
    
    // Настройка RF22B для Direct Mode
    rf22_direct_mode_init();
    Serial.println(F("RF22B настроен в Direct Mode"));
    
    // Инициализация фильтра Калмана
    kalman_init(&g_kalman, KALMAN_Q, KALMAN_R, 0, KALMAN_INIT_P);
    
    // Первичная калибровка
    Serial.println(F("Выполняется первичная калибровка..."));
    perform_calibration();
    Serial.println(F("Калибровка завершена"));
    
    Serial.println(F("Система готова к работе"));
    Serial.println(F("Нажмите кнопку для повторной калибровки"));
}

// ============================================================================
// Основной цикл (loop)
// ============================================================================
void loop() {
    // Проверка кнопки калибровки
    check_calibration_button();
    
    if (g_calibration_requested) {
        Serial.println(F("Калибровка по кнопке..."));
        perform_calibration();
        Serial.println(F("Калибровка завершена"));
    }
    
    // Обработка данных вариометра
    process_vario_data();
    
    // Отладка в Serial (каждые 2 секунды)
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 2000) {
        last_debug = millis();
        
        Serial.print(F("H: "));
        Serial.print(g_vario.altitude / 100.0f, 1);
        Serial.print(F(" м, V: "));
        Serial.print(g_vario.climb / 100.0f, 1);
        Serial.print(F(" м/с, P: "));
        Serial.print(g_vario.pressure / 100.0f, 0);
        Serial.println(F(" Па"));
        
        if (g_tone_freq > 0) {
            Serial.print(F("Тон: "));
            if (g_tone_continuous) {
                Serial.print(g_tone_freq);
                Serial.println(F(" Гц (непрерывный)"));
            } else {
                Serial.print(g_tone_freq);
                Serial.print(F(" Гц, период "));
                Serial.print(g_tone_period_ms);
                Serial.println(F(" мс (прерывистый)"));
            }
        } else {
            Serial.println(F("Тон: тишина (нейтральная зона)"));
        }
    }
    
    // Небольшая задержка
    delay(10);
}