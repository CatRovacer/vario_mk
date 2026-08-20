/*
  Звуковой Вариометр для nRF51822 / nRF52
  Датчик: BMP085 (I2C)
  Принцип работы: Эмуляция логики Brauniger IQ ONE
  Кнопка калибровки: Pin 13 (Pull-down, HIGH при нажатии)
  Пьезо-динамик: Pin 8 (через резистор 100 Ом или транзистор для громкости)
*/

#include <Wire.h>
//#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>

#define VERBOSE_ENABLED true

#ifdef VERBOSE_ENABLED
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif

//#define esp32c6_brd true
//#define esp32s3_brd true
//#define nrf52802_E73brd true
#define nrf51802_sens true
//#define nrf52822_R40sens true
//#define nrf52840_brd true 

#include "hardware_pins.h"


// === ПАРАМЕТРЫ ВАРИОМЕТРА ===
const float SMOOTHING_FACTOR = 0.35;  // Коэффициент сглаживания вертикальной скорости (меньше = плавнее)
const float KALMAN_NOISE = 2.0;       // Шум датчика для фильтра Калмана

// === НАСТРОЙКИ ЗВУКА (Brauniger style) ===
// Пороги вертикальной скорости (м/с)
const float CLIMB_THRESHOLD = 0.3;    // Выше этого начинаем пищать чаще (набор)
const float SINK_THRESHOLD = -0.3;    // Ниже этого пищим низким тоном (снижение)
const float ZERO_THRESHOLD = 0.1;     // Зона молчания +/- 0.1 м/с

// Частоты звука (Гц)
const int TONE_FAST_CLIMB = 1700;      // Быстрый набор -> Высокий тон
const int TONE_SLOW_CLIMB = 800;      // Медленный набор
const int TONE_SINK = 500;             // Снижение -> Низкий тон
const int TONE_FAST_SINK = 300;        // Быстрое снижение (прерывистый гудок)

// Параметры фильтра Калмана (масштабированные)
#define KALMAN_Q_SCALED     10    // Q * 1000
#define KALMAN_R_SCALED     500   // R * 1000
#define KALMAN_INIT_P       1000  // Начальная ковариация * 1000

static int32_t kalman_x = 0;          // состояние
static int32_t kalman_p = KALMAN_INIT_P; // ковариация * 1000
static int32_t kalman_k = 0;          // коэффициент Калмана * 1000

void kalman_init(int32_t initial_x) {
    kalman_x = initial_x;
    kalman_p = KALMAN_INIT_P;
    kalman_k = 0;
}

int32_t kalman_update(int32_t measurement) {
    // Прогноз: p = p + q
    kalman_p = kalman_p + KALMAN_Q_SCALED;
    if (kalman_p > 1000000) kalman_p = 1000000;
    
    // Коэффициент Калмана: k = p / (p + r)
    // Используем целочисленное деление с масштабированием
    int32_t denominator = kalman_p + KALMAN_R_SCALED;
    if (denominator == 0) denominator = 1;
    kalman_k = (kalman_p * 1000) / denominator;
    
    // Обновление состояния: x = x + k * (z - x)
    int32_t error = measurement - kalman_x;
    int32_t correction = (kalman_k * error) / 1000;
    kalman_x = kalman_x + correction;
    
    // Обновление ковариации: p = (1 - k) * p
    kalman_p = (kalman_p * (1000 - kalman_k)) / 1000;
    if (kalman_p < 1) kalman_p = 1;
    
    return kalman_x;
}

// Длительность импульсов (в миллисекундах)
// Чем выше скорость, тем короче пауза (звук сливается в сплошной при >5 м/с)
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

// === ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ===
//Adafruit_BMP085 BARO;
Adafruit_BMP280 BARO;

// Переменные для расчетов
float currentAltitude = 0;
float previousAltitude = 0;
float verticalSpeed = 0;
float filteredVerticalSpeed = 0;
unsigned long lastTime = 0;
bool isCalibrated = false;       // Флаг первой калибровки
float groundReference = 0;

// Таймер для "залипания" звука (Debounce для кнопки)
unsigned long lastButtonPress = 0;

// === ФУНКЦИЯ ПОДАЧИ ЗВУКА (Блокирующая, но для вариометра это нормально) ===
void playVariometerTone(float speed) {
  int frequency = 0;
  int duration = getBeepDuration(speed);
  bool isSound = true;
//    DBG_PRINT("vSp = ");    DBG_PRINTLN(speed);

  // 1. Зона молчания (терпимость)
  if (abs(speed) < ZERO_THRESHOLD) {
//    DBG_PRINTLN("Shut buzzer.");
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // 2. Логика выбора тона (Brauniger IQ ONE эмуляция)
  if (speed > CLIMB_THRESHOLD) {
    // НАБОР ВЫСОТЫ: Чем выше скорость, тем выше тон
    // Диапазон от 1500Гц (0.3м/с) до 2400Гц (5м/с)
    frequency = map(constrain(speed, 0.3, 5.0) * 100, 30, 500, TONE_SLOW_CLIMB, TONE_FAST_CLIMB);
    frequency = constrain(frequency, TONE_SLOW_CLIMB, TONE_FAST_CLIMB);
//    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Мерцание
  } 
  else if (speed < SINK_THRESHOLD) {
    // СНИЖЕНИЕ: Звук прерывистый, низкий. Чем быстрее падаем, тем ниже тон (страшнее)
    frequency = map(abs(constrain(speed, -5.0, -0.3) * 100), 30, 500, TONE_SINK, TONE_FAST_SINK);
    frequency = constrain(frequency, TONE_FAST_SINK, TONE_SINK);
    // Эффект мигания светодиодом в ритм снижения
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Мерцание
  }

  // Воспроизведение звука с учетом длительности паузы (Breaks)
  if (frequency > 0) {
    // Длительность звучания - половина общего времени импульса (чтобы были четкие паузы)
    int soundDuration = duration / 2;
    DBG_PRINT("\n*** playVariometerTone; freq: ");  DBG_PRINT(frequency); DBG_PRINT("; durat: "); DBG_PRINTLN(soundDuration);  
    tone(BUZZER_PIN, frequency, soundDuration);
    // Пауза между писками (оставшаяся часть времени + задержка на прерывистость)
    delay(duration);
  }
}

// === КАЛИБРОВКА НУЛЯ (Земля/Старт) ===
void calibrateZero() {
//  DBG_PRINTLN("***calibrateZero");
  // Берем 50 замеров для усреднения
  float sum = 0;

  kalman_init(0);

  for (int i = 0; i < 50; i++) {
    sum += BARO.readAltitude();
    delay(5);
  }
  groundReference = sum / 50.0;
  isCalibrated = true;
  
  // Звуковой сигнал подтверждения: 2 коротких писка
  tone(BUZZER_PIN, 2000, 100);
  delay(150);
  tone(BUZZER_PIN, 2500, 100);
  DBG_PRINT("*** calibrateZero; Reference pressure set to: ");
  DBG_PRINTLN(groundReference);
}

// === НАСТРОЙКА (SETUP) ===
void setup() {
  Serial.begin(115200);
//    while (!Serial) delay(10);

  DBG_PRINTLN("\n\n*** Vegetable Variometer mk2***");
    // Настройка пинов
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP); // Для nRF используем INPUT_PULLDOWN внутренний Pull-down если есть, иначе внешний резистор 10кОм на GND
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 5; i++){
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // Инициализация BMP085
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!BARO.begin()) {
    DBG_PRINTLN("*** setup; BMP085 init ERROR!");
    // Код ошибки: бесконечный писк
    while (1) {
      tone(BUZZER_PIN, 1000, 500);
      delay(500);
    }
  }
  
  DBG_PRINTLN("*** setup; BMP085 OK. Press button to set zero altitude.");
  
  // Ожидание калибровки перед стартом
  calibrateZero();
#if 1
  while (!isCalibrated) {
    if (digitalRead(MODE_BUTTON_PIN) == HIGH) {
      delay(50); // Антидребезг
      if (digitalRead(MODE_BUTTON_PIN) == HIGH) {
        calibrateZero();
      }
    }
    delay(100);
    // Моргаем пока не откалиброван
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
#endif
  // Стартовые значения
  previousAltitude = BARO.readAltitude();
  lastTime = millis();
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

// === ГЛАВНЫЙ ЦИКЛ ===
void loop() {
  // 1. Обработка кнопки (Калибровка на лету / Сброс)
  if (digitalRead(MODE_BUTTON_PIN) == LOW) {
    if (millis() - lastButtonPress > 500) { // Защита от ложных срабатываний
      calibrateZero(); // Сбрасываем текущую высоту в ноль
      lastButtonPress = millis();
      // Сброс фильтра Калмана для плавного перехода
      // Пересоздаем объект или сбрасываем состояние (в SimpleKalmanFilter сброс через new, но проще обнулить переменные)
      previousAltitude = groundReference; 
      verticalSpeed = 0;
      filteredVerticalSpeed = 0;
    }
  }

  // 2. Чтение сырых данных и фильтрация
  // Получаем сырую высоту от BMP (она сама компенсирует температуру)
  float rawAltitude = BARO.readAltitude();
  
  // Фильтр Калмана убирает высокочастотные шумы (болтанку)
  float kalmanAltitude = kalman_update(rawAltitude);
// kalmanFilter.updateEstimate(rawAltitude);
  
  // Абсолютная высота относительно точки калибровки
  currentAltitude = kalmanAltitude - groundReference;
  
  // 3. Расчет вертикальной скорости (dV/dt)
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
//  DBG_PRINT("*** loop; dt = "); DBG_PRINTLN(dt); 
  if (dt >= 0.05 && dt < 0.100) { // Расчет с частотой ~20 Гц
    float deltaAlt = currentAltitude - previousAltitude;
    float instantSpeed = deltaAlt / dt;
    
    // Экспоненциальное сглаживание (аналог Brauniger)
    // filteredSpeed = (instantSpeed * FACTOR) + (filteredSpeed * (1-FACTOR))
    filteredVerticalSpeed = (instantSpeed * SMOOTHING_FACTOR) + (filteredVerticalSpeed * (1.0 - SMOOTHING_FACTOR));
    
    verticalSpeed = filteredVerticalSpeed;
    previousAltitude = currentAltitude;
    lastTime = now;
    
    // Вывод в Serial монитор для отладки
    DBG_PRINT("*** loop; Alt:"); DBG_PRINT(currentAltitude, 2);
    DBG_PRINT("m | V:"); DBG_PRINT(verticalSpeed, 2);
    DBG_PRINTLN("m/s");
  }

  // 4. Звуковая индикация
  playVariometerTone(verticalSpeed);
  
  // Небольшая задержка для стабильности цикла (основная задержка внутри playVariometerTone)
  // Если звук не играет, ставим 50мс, чтобы не грузить процессор
  if (abs(verticalSpeed) < ZERO_THRESHOLD) {
    delay(50);
  }
}
