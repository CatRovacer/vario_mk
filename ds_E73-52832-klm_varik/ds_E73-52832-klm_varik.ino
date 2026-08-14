/*
  Вариометр на nRF52832 (E73) + BMP280
  - Фильтр Калмана для высоты
  - Звук изменяется пропорционально вертикальной скорости
  - Кнопка калибровки (обнуление высоты)
  - Аналог Brauniger IQ ONE
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// ==================== НАСТРОЙКИ ====================
#define PIN_BUZZER      17    // Пьезо-пищалка на GPIO 17
#define PIN_BUTTON      18    // Кнопка калибровки (подтяжка к GND)

#define ALT_HISTORY     10    // Глубина истории для расчёта скорости
#define UPDATE_DELAY    50    // Обновление каждые 50 мс (20 Гц)
#define SOUND_MIN_HZ    200   // Минимальная частота тона (при пикировании)
#define SOUND_MAX_HZ    2000  // Максимальная частота тона (при наборе)
#define SOUND_DEADZONE  0.3   // Мёртвая зона в м/с (звук не меняется)

// Коэффициенты фильтра Калмана (подобраны для высоты)
#define KALMAN_Q        0.01  // Шум процесса
#define KALMAN_R        1.0   // Шум измерений

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
Adafruit_BMP280 bmp;

float groundPressure = 0.0;      // Давление на уровне калибровки (гПа)
float kalmanXe = 0.0;            // Оценка состояния (высота)
float kalmanP = 1.0;             // Ковариация ошибки
float altitudeHistory[ALT_HISTORY];
byte historyIndex = 0;
bool historyFilled = false;
unsigned long lastUpdate = 0;

// ==================== ФИЛЬТР КАЛМАНА ====================
float kalmanFilter(float measurement) {
  // Прогноз: Xp = Xe, Pp = P + Q
  float Xp = kalmanXe;
  float Pp = kalmanP + KALMAN_Q;
  
  // Коррекция: G = Pp / (Pp + R)
  float G = Pp / (Pp + KALMAN_R);
  
  // Обновление: Xe = Xp + G * (measurement - Xp)
  kalmanXe = Xp + G * (measurement - Xp);
  
  // Обновление ковариации: P = (1 - G) * Pp
  kalmanP = (1.0 - G) * Pp;
  
  return kalmanXe;
}

// ==================== ВЫЧИСЛЕНИЕ ВЕРТИКАЛЬНОЙ СКОРОСТИ ====================
float calculateVerticalSpeed(float currentAltitude) {
  // Сдвиг истории
  if (historyFilled) {
    for (byte i = 0; i < ALT_HISTORY - 1; i++) {
      altitudeHistory[i] = altitudeHistory[i + 1];
    }
    altitudeHistory[ALT_HISTORY - 1] = currentAltitude;
  } else {
    altitudeHistory[historyIndex++] = currentAltitude;
    if (historyIndex >= ALT_HISTORY) {
      historyFilled = true;
      historyIndex = 0;
    }
    return 0.0;
  }
  
  // Линейная регрессия по истории
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  float dt = UPDATE_DELAY / 1000.0;
  
  for (byte i = 0; i < ALT_HISTORY; i++) {
    float x = i * dt;
    float y = altitudeHistory[i];
    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumX2 += x * x;
  }
  
  float denominator = ALT_HISTORY * sumX2 - sumX * sumX;
  if (denominator == 0) return 0.0;
  
  return (ALT_HISTORY * sumXY - sumX * sumY) / denominator; // м/с
}

// ==================== ГЕНЕРАТОР ТОНА (PWM) ====================
void setTone(float verticalSpeed) {
  // Ограничиваем скорость для расчёта частоты
  float limitedSpeed = constrain(abs(verticalSpeed), 0.1, 10.0);
  
  // Расчёт частоты: экспоненциальная шкала (как в Brauniger)
  // При скорости 0.1 м/с → SOUND_MIN_HZ, при 10 м/с → SOUND_MAX_HZ
  float normalized = (log10(limitedSpeed) - log10(0.1)) / (log10(10.0) - log10(0.1));
  float freq;
  
  if (verticalSpeed > SOUND_DEADZONE) {
    // Набор высоты: частота растёт
    freq = SOUND_MIN_HZ + normalized * (SOUND_MAX_HZ - SOUND_MIN_HZ);
  } else if (verticalSpeed < -SOUND_DEADZONE) {
    // Снижение: частота падает (инвертированная шкала)
    freq = SOUND_MAX_HZ - normalized * (SOUND_MAX_HZ - SOUND_MIN_HZ);
  } else {
    // Мёртвая зона
    noTone(PIN_BUZZER);
    return;
  }
  
  // Ограничение диапазона
  freq = constrain(freq, SOUND_MIN_HZ, SOUND_MAX_HZ);
  
  // Генерация тона (Arduino-совместимая функция)
  tone(PIN_BUZZER, freq);
}

// ==================== КАЛИБРОВКА НУЛЯ ====================
void calibrateZero() {
  float sumPressure = 0;
  
  // Усреднение 50 измерений для стабильности
  for (int i = 0; i < 50; i++) {
    sumPressure += bmp.readPressure();
    delay(5);
  }
  
  groundPressure = sumPressure / 50.0 / 100.0; // Па → гПа
  
  // Сброс фильтра Калмана
  kalmanXe = 0.0;
  kalmanP = 1.0;
  
  // Сброс истории
  historyIndex = 0;
  historyFilled = false;
  
  // Сигнал подтверждения калибровки
  tone(PIN_BUZZER, 1000, 200);
  delay(250);
  tone(PIN_BUZZER, 1500, 200);
}

// ==================== ИНИЦИАЛИЗАЦИЯ ====================
void setup() {
  Serial.begin(115200);
  
  // Настройка пинов
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  
  // Инициализация BMP280
  Wire.begin();
  if (!bmp.begin(0x76)) {  // Адрес 0x76 (часто используется) или 0x77
    Serial.println("BMP280 не найден! Проверьте подключение.");
    while (1) {
      digitalWrite(PIN_BUZZER, HIGH);
      delay(100);
      digitalWrite(PIN_BUZZER, LOW);
      delay(900);
    }
  }
  
  // Настройка BMP280: oversampling x2, фильтр x4
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,  // Температура
                  Adafruit_BMP280::SAMPLING_X2,  // Давление
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_1);
  
  delay(100);
  
  // Начальная калибровка
  calibrateZero();
  
  Serial.println("Вариометр готов!");
  Serial.println("Кнопка: калибровка нуля");
  Serial.println("---");
  
  lastUpdate = millis();
}

// ==================== ОСНОВНОЙ ЦИКЛ ====================
void loop() {
  // Проверка кнопки калибровки
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(PIN_BUTTON);
  
  if (lastButtonState == HIGH && buttonState == LOW) {
    calibrateZero();
    delay(200);  // Антидребезг
  }
  lastButtonState = buttonState;
  
  // Обновление по таймеру
  if (millis() - lastUpdate >= UPDATE_DELAY) {
    // Чтение давления (гПа)
    float pressure = bmp.readPressure() / 100.0;
    
    if (groundPressure > 0) {
      // Расчёт высоты по барометрической формуле
      float rawAltitude = 44330.0 * (1.0 - pow(pressure / groundPressure, 1.0 / 5.255));
      
      // Фильтр Калмана
      float filteredAltitude = kalmanFilter(rawAltitude);
      
      // Расчёт вертикальной скорости
      float verticalSpeed = calculateVerticalSpeed(filteredAltitude);
      
      // Генерация звука
      setTone(verticalSpeed);
      
      // Вывод в Serial для отладки
      Serial.print("Высота: ");
      Serial.print(filteredAltitude, 1);
      Serial.print(" м  |  Скорость: ");
      Serial.print(verticalSpeed, 2);
      Serial.print(" м/с  |  Частота: ");
      if (abs(verticalSpeed) > SOUND_DEADZONE) {
        float freq = (verticalSpeed > 0) ? 
          map(min(abs(verticalSpeed), 10.0) * 100, 10, 1000, SOUND_MIN_HZ, SOUND_MAX_HZ) :
          map(min(abs(verticalSpeed), 10.0) * 100, 10, 1000, SOUND_MAX_HZ, SOUND_MIN_HZ);
        Serial.print(constrain(freq, SOUND_MIN_HZ, SOUND_MAX_HZ));
      } else {
        Serial.print("---");
      }
      Serial.println(" Hz");
    }
    
    lastUpdate = millis();
  }
}