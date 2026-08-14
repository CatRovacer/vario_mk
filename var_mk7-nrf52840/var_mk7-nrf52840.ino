/*
  Вариометр для nRF52840 (XIAO BLE) с датчиком DPS310
  Логика звука: Brauniger IQ ONE style
  - Подъем: Частота тона растет, паузы между импульсами уменьшаются (трель).
  - Снижение: Низкий тон, паузы увеличиваются пропорционально скорости.
*/

#include <Wire.h>

#define ADAFRUIT_DPS310_LIB true
#define DPS310_LIB      false
#define LPS22HB_LIB	false

#if ADAFRUIT_DPS310_LIB
#include <Adafruit_DPS310.h>
#elif DPS310_LIB
#include <DPS310.h>
#elif LPS22HB_LIB
#include <Arduino_LPS22HB.h>
#endif

#define VERBOSE_ENABLED false
#ifdef VERBOSE_ENABLED
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif


// ==================== НАСТРОЙКИ ====================
#define DPS310_ADDRESS 0x77          // I2C адрес датчика (обычно 0x77)
#define SAMPLE_RATE_HZ 10            // Частота обновления (Гц)
#define SAMPLE_INTERVAL_MS 100       // Интервал измерения (1000 / SAMPLE_RATE_HZ)
#define SMOOTHING_FACTOR 0.15        // Коэффициент EMA фильтра (меньше = сильнее сглаживание)
#define VARIO_DEADBAND 0.05          // Мертвая зона (м/с) – тишина при зависании

// Настройки звука (пин, таймер, частоты)
#define TIMER_FREQ_HZ 10000          // Частота таймера (10kHz)
#define CLIMB_FREQ_MIN 300           // Мин частота звука при подъеме (Гц)
#define CLIMB_FREQ_MAX 1500          // Макс частота звука при подъеме (Гц)
#define SINK_FIXED_FREQ 256          // Фиксированная низкая частота при снижении (Гц)
#define MAX_VARIO_RATE 6.0           // Максимальная отображаемая скорость (м/с)

// Настройки кнопки сброса
#define BUZZER_PIN A0                // Пин для пассивного зуммера (PWM)
#define MODE_BUTTON_PIN A1                // Пин кнопки (Pull-up внутренний)
#define LONG_PRESS_MS 2000           // Время удержания для сброса высоты (мс)

// Параметры звукового импульса (PWM и таймер)
#define TONE_DUTY_CYCLE 0.5          // Скважность 50%
#define TONE_DURATION_MS 60          // Длительность импульса "бип" (мс)

// Параметры звука Brauniger IQ ONE
#define SOUND_NEUTRAL_ZONE    10           // см/с - нейтральная зона
#define SOUND_MIN_FREQ        CLIMB_FREQ_MIN          // Гц
#define SOUND_MAX_FREQ        CLIMB_FREQ_MAX         // Гц
#define SOUND_CLIMB_BASE      600          // Гц
#define SOUND_SINK_BASE       600          // Гц

#define MIN_BEEP_MS           50   // Минимальный период звучания (мс) (при быстром подъёме)
#define MAX_BEEP_MS           800  // Максимальный период (мс) (при быстром снижении)

#define SIMULATION_ENABLED    true    // Режим эмуляции для отладки
#define SIMULATION_PERIOD_MS  1000    // Период изменения скорости при эмуляции

// ===================================================

Adafruit_DPS310 dps;

// Переменные фильтрации
float filtered_altitude = 0.0;
float last_altitude = 0.0;
unsigned long last_sample_time = 0;
unsigned long last_beep_time = 0;
float vario_speed = 0.0;

// Переменные UI (Кнопка)
unsigned long button_press_start = 0;
bool button_was_pressed = false;
float altitude_offset = 0.0;         // Смещение высоты (относительная высота)


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

// Переменные звукового синтеза (Timer/Counter)
// Для nRF52840 используем таймер 2 (64MHz) для генерации неблокирующего тона
NRF_TIMER_Type* toneTimer = NRF_TIMER2;
volatile uint32_t tone_cc_value = 0;
volatile bool tone_active = false;
volatile uint32_t tone_cycles_on = 0;
volatile uint32_t tone_cycles_off = 0;
volatile uint32_t tone_phase = 0;

// ==================== НАСТРОЙКА ТАЙМЕРА ДЛЯ ТОНА ====================
void setupToneTimer() {
  // Включаем тактирование таймера 2
  NRF_CLOCK->TASKS_HFCLKSTART = 1;  // Запускаем High Frequency Clock
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED);
  
  toneTimer->MODE = TIMER_MODE_MODE_Timer;
  toneTimer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  toneTimer->PRESCALER = 6;  // 64MHz / 2^6 = 1MHz (1us тик)
  toneTimer->CC[0] = 0;
  
  // Настройка задачи сравнения 0 (Clear/Set)
  toneTimer->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Msk | TIMER_SHORTS_COMPARE0_STOP_Msk);
  
  // Включаем прерывание по COMPARE0
  toneTimer->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
  NVIC_EnableIRQ(TIMER2_IRQn);
  
  toneTimer->TASKS_START = 1;
}

// Прерывание таймера для генерации меандра (неблокирующий tone)
extern "C" void TIMER2_IRQHandler(void) {
  if (toneTimer->EVENTS_COMPARE[0] && toneTimer->INTENSET & TIMER_INTENSET_COMPARE0_Msk) {
    toneTimer->EVENTS_COMPARE[0] = 0;
    
    if (tone_active) {
      // Инвертируем состояние пина
      nrf_gpio_pin_toggle(BUZZER_PIN);
      
      // Устанавливаем следующую задержку (полупериод)
      if (nrf_gpio_pin_read(BUZZER_PIN) == HIGH) {
        toneTimer->CC[0] = tone_cycles_on;
      } else {
        toneTimer->CC[0] = tone_cycles_off;
      }
    } else {
      // Если тон не активен, убеждаемся, что пин LOW
      nrf_gpio_pin_write(BUZZER_PIN, LOW);
      toneTimer->CC[0] = 10000; // "Пустой" период, не 0 чтобы не спамить прерывания
    }
    toneTimer->TASKS_START = 1; // Перезапуск для следующего цикла
  }
}

// Вспомогательная переменная для управления длительностью тона в основном цикле
unsigned long tone_duration_end = 0;

// Функция запуска тона с заданной частотой и длительностью (НЕБЛОКИРУЮЩАЯ)
void startTone(float freq_hz, unsigned long duration_ms) {
  if (freq_hz <= 0) {
    stopTone();
    return;
  }
  
  uint32_t half_period_us = (uint32_t)(500000.0 / freq_hz); // Полупериод в микросекундах (1MHz тики)
  if (half_period_us < 2) half_period_us = 2; // Ограничение
  
  tone_cycles_on = half_period_us;
  tone_cycles_off = half_period_us;
  
  // Сбрасываем фазу
  tone_phase = 0;
  tone_active = true;
  
  // Устанавливаем начальное состояние
  nrf_gpio_pin_write(BUZZER_PIN, HIGH);
  toneTimer->CC[0] = tone_cycles_on;
  toneTimer->TASKS_START = 1;
  
  last_beep_time = millis();
  tone_duration_end = last_beep_time + duration_ms;
}

void stopTone() {
  tone_active = false;
  nrf_gpio_pin_write(BUZZER_PIN, LOW);
}

// Проверка истечения длительности тона (вызывается в loop)
void updateToneDuration() {
  if (tone_active && millis() >= tone_duration_end) {
    stopTone();
  }
}

// ==================== ИНИЦИАЛИЗАЦИЯ DPS310 ====================
bool initDPS310() {
  if (!dps.begin_I2C(DPS310_ADDRESS)) {
    Serial.println("Ошибка: DPS310 не найден!");
    return false;
  }
  
  // Настройка высокой точности для вариометра
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  
  Serial.println("DPS310 инициализирован!");
  return true;
}

// Чтение высоты в метрах (с использованием барометрической формулы)
float readAltitude() {
  sensors_event_t temp_event, pressure_event;
  dps.getEvents(&temp_event, &pressure_event);
  
  // Базовое давление (1013.25 hPa) - должно быть установлено через калибровку
  float pressure_hPa = pressure_event.pressure;
  // Простая формула пересчета давления в высоту (для старта)
  // Более точная формула: altitude = 44330 * (1 - pow(pressure / seaLevelPressure, 0.1903))
  float altitude = 44330.0 * (1.0 - pow(pressure_hPa / 1013.25, 0.1903));
  
  // Применяем смещение (пользовательская калибровка нуля)
  return altitude - altitude_offset;
}

// ==================== EMA ФИЛЬТР ====================
float exponentialMovingAverage(float newValue, float oldValue, float alpha) {
  return (alpha * newValue) + ((1.0 - alpha) * oldValue);
}

// ==================== РАСЧЕТ ВАРИО ====================
float calculateVario(float current_alt, float last_alt, float dt_sec) {
  if (dt_sec <= 0) return 0.0;
  float raw_vario = (current_alt - last_alt) / dt_sec;
  // Ограничение скорости для стабильности
  if (raw_vario > MAX_VARIO_RATE) raw_vario = MAX_VARIO_RATE;
  if (raw_vario < -MAX_VARIO_RATE) raw_vario = -MAX_VARIO_RATE;
  return raw_vario;
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

// ==================== ЗВУКОВАЯ ЛОГИКА (BRAUNIGER STYLE) ====================
void updateAudio(float vario) {
  float abs_vario = abs(vario);
  
  // Мертвая зона (тишина)
  if (abs_vario <= VARIO_DEADBAND) {
    stopTone();
    return;
  }
  
  // Расчет карты частоты и периода импульсов
  float t_norm = constrain(abs_vario / MAX_VARIO_RATE, 0.0, 1.0);
  
  if (vario > 0) {
    // ---------- РЕЖИМ ПОДЪЕМА (Climb) ----------
    // Частота тона растет: 300Hz -> 1500Hz
    float freq = CLIMB_FREQ_MIN + (CLIMB_FREQ_MAX - CLIMB_FREQ_MIN) * t_norm;
    // Интервал между "бип-бип" уменьшается: Максимум 600ms, Минимум 40ms (эффект трели)
    unsigned long period_ms = map(t_norm * 100, 0, 100, 600, 40);
    
    // Воспроизводим импульс длительностью 60мс. Пауза = period_ms - 60.
    // Чтобы это работало неблокирующе, мы просто перезапускаем тон каждые period_ms.
    if (millis() - last_beep_time >= period_ms) {
      startTone(freq, TONE_DURATION_MS);
    }
    
  } else {
    // ---------- РЕЖИМ СНИЖЕНИЯ (Sink) ----------
    // Частота тона низкая и постоянная
    float freq = SINK_FIXED_FREQ;
    // Интервал между гудками РАСТЕТ с ростом скорости снижения: 300ms -> 1500ms
    unsigned long period_ms = map(t_norm * 100, 0, 100, 300, 1500);
    
    if (millis() - last_beep_time >= period_ms) {
      // При сильном снижении делаем длинный гудок, при слабом - короткий
      unsigned long duration = map(t_norm * 100, 0, 100, 100, 400);
      startTone(freq, duration);
    }
  }
}

// ==================== КНОПКА КАЛИБРОВКИ НУЛЯ ====================
void handleButton() {
  bool current_state = digitalRead(MODE_BUTTON_PIN);
  
  if (current_state == LOW && !button_was_pressed) {
    // Кнопка нажата
    button_press_start = millis();
    button_was_pressed = true;
  }
  else if (current_state == LOW && button_was_pressed) {
    // Удерживается
    if ((millis() - button_press_start) >= LONG_PRESS_MS) {
      // Долгое нажатие: Сброс высоты
      float current_alt = readAltitude(); // Читаем текущую абсолютную высоту
      altitude_offset = current_alt;      // Устанавливаем смещение так, чтобы высота стала 0
      Serial.println("Высота обнулена!");
      
      // Небольшая обратная связь: 3 коротких писка
      stopTone();
      delay(50);
      startTone(1000, 100);
      delay(150);
      startTone(1000, 100);
      delay(150);
      startTone(1000, 100);
      
      button_was_pressed = false; // Сбрасываем флаг, чтобы не повторять
    }
  }
  else if (current_state == HIGH && button_was_pressed) {
    // Кнопка отпущена (без долгого удержания) - можно использовать для других функций
    button_was_pressed = false;
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Настройка пинов
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  
  // Инициализация датчика
  if (!initDPS310()) {
    while (1) {
      // Ошибка: пищим раз в секунду
      tone(BUZZER_PIN, 400, 500);
      delay(1000);
    }
  }
  
  // Первый замер для инициализации фильтра
  filtered_altitude = readAltitude();
  last_altitude = filtered_altitude;
  last_sample_time = millis();
  
  // Настройка высокоточного звукового таймера
  setupToneTimer();
  
  Serial.println("Вариометр запущен. Ждите...");
  delay(1000);
  
  // Приветственный сигнал
  startTone(880, 200);
  delay(300);
  startTone(880, 200);
}

// ==================== LOOP ====================
void loop() {
  handleButton();          // Обработка кнопки сброса
  updateToneDuration();    // Отключение звука по таймауту (работает параллельно)
  
  // Управление частотой опроса
  if (millis() - last_sample_time >= SAMPLE_INTERVAL_MS) {
    float dt = (millis() - last_sample_time) / 1000.0;
    last_sample_time = millis();
    
    // Чтение сырой высоты
    float raw_altitude = readAltitude();
    
    // Фильтрация шумов (EMA)
    filtered_altitude = exponentialMovingAverage(raw_altitude, filtered_altitude, SMOOTHING_FACTOR);
    
    // Расчет скорости (варио)
    vario_speed = calculateVario(filtered_altitude, last_altitude, dt);
    last_altitude = filtered_altitude;
    
    // Обновление звука на основе текущей скорости
    updateAudio(vario_speed);
    
    // Вывод в Serial Monitor (для отладки)
    Serial.print("Alt: "); Serial.print(filtered_altitude, 2); Serial.print(" m | ");
    Serial.print("Vario: "); Serial.print(vario_speed, 2); Serial.print(" m/s | ");
    if (vario_speed > 0) Serial.println("UP >>>");
    else if (vario_speed < 0) Serial.println("DOWN <<<");
    else Serial.println("--");
  }
}