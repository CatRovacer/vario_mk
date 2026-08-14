/*
 * Variometer для nRF52840 (nRF52810) с BMP180
 * Аналог Brauniger IQ One
 * 
 * Функции:
 * - Измерение высоты и вертикальной скорости с фильтром Калмана
 * - Тональные прерывистые звуковые сигналы (PWM) в зависимости от скорости
 * - Передача данных по Serial и BLE в формате LK8EX1 для XCSoar/LK8000
 * - Долгое нажатие (2 сек) - сброс точки отсчета высоты
 * - Двойное нажатие - отключение звука
 * - Режим симуляции вертикальной скорости (для отладки)
 * - Поддержка OTA (требуется настройка загрузчика)
 * 
 * Подключение:
 * BMP180: SDA -> A4/SDA, SCL -> A5/SCL, VCC -> 3.3V, GND -> GND
 * Кнопка: Pin 6 -> GND (внутренний подтягивающий резистор)
 * Пьезо-пищалка: Pin 5 -> через резистор 100 Ом -> +
 */

#include <Wire.h>
#include <SFE_BMP180.h>
#include <ArduinoBLE.h>

// ==================== НАСТРОЙКИ ====================
#define BUTTON_PIN 6
#define BUZZER_PIN 5
#define SIMULATION_PIN 7          // замкнуть на GND для входа в режим симуляции

// Параметры звука (аналог Brauniger IQ One)
#define BASE_FREQ_UP 1200         // частота при подъеме (Гц)
#define BASE_FREQ_DOWN 600        // частота при спуске (Гц)
#define FREQ_PER_MS 80            // изменение частоты на 1 м/с
#define MIN_BEEP_INTERVAL 50      // минимальный интервал между сигналами (мс)
#define MAX_BEEP_INTERVAL 2000    // максимальный интервал (мс)

// Параметры фильтра Калмана
#define KALMAN_Q 0.001            // шум процесса
#define KALMAN_R 0.1              // шум измерения

// Параметры LK8EX1
#define PRESSURE_SEA_LEVEL 1013.25 // давление на уровне моря (гПа)

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
SFE_BMP180 bmp180;

// Фильтр Калмана для высоты
float kalman_X = 0;               // оценка состояния (высота)
float kalman_P = 1;               // ковариация ошибки
float kalman_Q = KALMAN_Q;
float kalman_R = KALMAN_R;

// Данные
float altitude = 0;               // относительная высота (м)
float verticalSpeed = 0;          // вертикальная скорость (м/с)
float pressure = 0;              // давление (гПа)
float temperature = 0;           // температура (°C)
float baselinePressure = 1013.25; // опорное давление для высоты

// Управление звуком
bool soundEnabled = true;
unsigned long lastBeepTime = 0;
float lastAltitude = 0;
unsigned long lastAltitudeTime = 0;

// Управление кнопкой
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
int clickCount = 0;
unsigned long lastClickTime = 0;
#define DOUBLE_CLICK_TIMEOUT 300   // мс
#define LONG_PRESS_TIME 2000      // мс

// Режим симуляции
bool simulationMode = false;
float simVerticalSpeed = 0;
unsigned long simLastTime = 0;
float simAltitude = 0;

// BLE
BLEService varioService("180A");  // стандартный сервис Device Information
BLECharacteristic txChar("2A6E", BLERead | BLENotify, 64); // TX
BLECharacteristic rxChar("2A6F", BLEWrite, 64);           // RX

// ==================== ФИЛЬТР КАЛМАНА ====================
float kalmanUpdate(float measurement) {
  // Предсказание
  kalman_P = kalman_P + kalman_Q;
  
  // Коррекция
  float K = kalman_P / (kalman_P + kalman_R);
  kalman_X = kalman_X + K * (measurement - kalman_X);
  kalman_P = (1 - K) * kalman_P;
  
  return kalman_X;
}

// ==================== РАСЧЕТ ВЕРТИКАЛЬНОЙ СКОРОСТИ ====================
float calculateVerticalSpeed(float currentAlt, unsigned long currentTime) {
  static float filteredSpeed = 0;
  const float alpha = 0.3; // коэффициент сглаживания
  
  if (lastAltitudeTime == 0) {
    lastAltitude = currentAlt;
    lastAltitudeTime = currentTime;
    return 0;
  }
  
  float dt = (currentTime - lastAltitudeTime) / 1000.0; // секунды
  if (dt < 0.1) return filteredSpeed; // слишком малое время
  
  float rawSpeed = (currentAlt - lastAltitude) / dt;
  
  // Сглаживание (дополнительный low-pass фильтр)
  filteredSpeed = alpha * rawSpeed + (1 - alpha) * filteredSpeed;
  
  lastAltitude = currentAlt;
  lastAltitudeTime = currentTime;
  
  return filteredSpeed;
}

// ==================== ГЕНЕРАЦИЯ ЗВУКА ====================
void generateVarioSound(float speed) {
  if (!soundEnabled) return;
  
  unsigned long now = millis();
  
  // Определяем интервал между сигналами в зависимости от скорости
  float absSpeed = abs(speed);
  unsigned long interval;
  
  if (absSpeed < 0.1) {
    // Парящий режим - редкие сигналы
    interval = 1500;
  } else {
    // Чем больше скорость, тем чаще сигналы
    interval = mapFloat(absSpeed, 0.1, 10.0, 1200, 100);
    interval = constrain(interval, MIN_BEEP_INTERVAL, MAX_BEEP_INTERVAL);
  }
  
  // Определяем частоту тона
  int frequency;
  if (speed > 0) {
    // Подъем - высокий тон, частота растет со скоростью
    frequency = BASE_FREQ_UP + speed * FREQ_PER_MS;
    frequency = constrain(frequency, BASE_FREQ_UP, 2500);
  } else {
    // Спуск - низкий тон, частота падает со скоростью
    frequency = BASE_FREQ_DOWN + abs(speed) * FREQ_PER_MS;
    frequency = constrain(frequency, 200, BASE_FREQ_DOWN);
  }
  
  // Длительность сигнала
  int duration = constrain(100 - absSpeed * 5, 20, 100);
  
  // Проверяем, пора ли пищать
  if (now - lastBeepTime >= interval) {
    // Прерывистый сигнал: для подъема - серия коротких сигналов
    if (speed > 0.5) {
      // Подъем: 3 коротких сигнала
      for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, frequency, duration);
        delay(duration + 30);
      }
    } else if (speed < -0.5) {
      // Спуск: 1 длинный сигнал
      tone(BUZZER_PIN, frequency, duration * 3);
      delay(duration * 3);
    } else {
      // Парящий режим: короткий сигнал
      tone(BUZZER_PIN, frequency, duration);
    }
    lastBeepTime = now;
  }
}

// ==================== ФОРМИРОВАНИЕ LK8EX1 ====================
String buildLK8EX1(float press, float alt, float vario, float temp, float batt) {
  // Формат: $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
  String sentence = "$LK8EX1,";
  sentence += String(press, 2) + ",";
  sentence += String(alt, 2) + ",";
  sentence += String(vario, 2) + ",";
  sentence += String(temp, 1) + ",";
  sentence += String(batt, 1);
  
  // Расчет контрольной суммы (XOR всех байтов между $ и *)
  byte checksum = 0;
  for (int i = 1; i < sentence.length(); i++) {
    checksum ^= sentence.charAt(i);
  }
  
  sentence += "*" + String(checksum, HEX);
  if (sentence.length() < 4) sentence = "0" + sentence;
  sentence += "\r\n";
  
  return sentence;
}

// ==================== ОБРАБОТКА КНОПКИ ====================
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN) == LOW;
  unsigned long now = millis();
  
  if (reading && !buttonPressed) {
    // Кнопка нажата
    buttonPressed = true;
    buttonPressTime = now;
  } else if (!reading && buttonPressed) {
    // Кнопка отпущена
    buttonPressed = false;
    unsigned long pressDuration = now - buttonPressTime;
    
    if (pressDuration >= LONG_PRESS_TIME) {
      // Долгое нажатие (> 2 сек) - сброс высоты
      baselinePressure = pressure;
      kalman_X = 0;
      kalman_P = 1;
      altitude = 0;
      verticalSpeed = 0;
      Serial.println("Altitude reset");
      // Короткий звук подтверждения
      tone(BUZZER_PIN, 1000, 200);
      delay(200);
      tone(BUZZER_PIN, 1500, 200);
    } else if (pressDuration < DOUBLE_CLICK_TIMEOUT) {
      // Короткое нажатие - считаем клики
      if (now - lastClickTime < DOUBLE_CLICK_TIMEOUT) {
        clickCount++;
      } else {
        clickCount = 1;
      }
      lastClickTime = now;
      
      if (clickCount >= 2) {
        // Двойное нажатие - отключение звука
        soundEnabled = !soundEnabled;
        Serial.print("Sound: ");
        Serial.println(soundEnabled ? "ON" : "OFF");
        // Звук подтверждения
        if (soundEnabled) {
          tone(BUZZER_PIN, 1500, 100);
        } else {
          tone(BUZZER_PIN, 500, 200);
        }
        clickCount = 0;
      }
    }
  }
  
  // Сброс счетчика кликов по таймауту
  if (clickCount > 0 && (now - lastClickTime > DOUBLE_CLICK_TIMEOUT)) {
    clickCount = 0;
  }
}

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ==================== НАСТРОЙКА BLE ====================
void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
  
  BLE.setDeviceName("Variometer");
  BLE.setLocalName("Variometer");
  BLE.setAdvertisedService(varioService);
  
  varioService.addCharacteristic(txChar);
  varioService.addCharacteristic(rxChar);
  
  BLE.addService(varioService);
  
  // Начинаем рекламироваться
  BLE.advertise();
  Serial.println("BLE ready");
}

// ==================== НАСТРОЙКА ====================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("Variometer starting...");
  
  // Настройка пинов
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SIMULATION_PIN, INPUT_PULLUP);
  
  // Инициализация BMP180
  Wire.begin();
  if (!bmp180.begin()) {
    Serial.println("BMP180 not found!");
    while (1);
  }
  Serial.println("BMP180 found");
  
  // Проверка режима симуляции
  if (digitalRead(SIMULATION_PIN) == LOW) {
    simulationMode = true;
    Serial.println("SIMULATION MODE");
    simLastTime = millis();
  }
  
  // Инициализация BLE
  setupBLE();
  
  // Первое измерение для инициализации фильтра
  char status;
  double T, P;
  status = bmp180.startTemperature();
  if (status != 0) {
    delay(status);
    status = bmp180.getTemperature(T);
    if (status != 0) {
      status = bmp180.startPressure(3);
      if (status != 0) {
        delay(status);
        status = bmp180.getPressure(P, T);
        if (status != 0) {
          pressure = P;
          temperature = T;
          float alt = bmp180.altitude(P, PRESSURE_SEA_LEVEL);
          kalman_X = alt;
          altitude = 0;
          baselinePressure = P;
        }
      }
    }
  }
  
  Serial.println("Ready!");
  tone(BUZZER_PIN, 2000, 100);
  delay(150);
  tone(BUZZER_PIN, 2500, 100);
}

// ==================== ОСНОВНОЙ ЦИКЛ ====================
void loop() {
  static unsigned long lastReadTime = 0;
  static unsigned long lastTransmitTime = 0;
  const unsigned long READ_INTERVAL = 50;   // 20 Гц
  const unsigned long TRANSMIT_INTERVAL = 100; // 10 Гц
  
  unsigned long now = millis();
  
  // Обработка кнопки
  handleButton();
  
  // BLE: обработка входящих команд (для OTA)
  BLE.poll();
  
  // Чтение датчика
  if (now - lastReadTime >= READ_INTERVAL) {
    lastReadTime = now;
    
    if (simulationMode) {
      // ===== РЕЖИМ СИМУЛЯЦИИ =====
      float dt = (now - simLastTime) / 1000.0;
      simLastTime = now;
      
      // Изменяем скорость синусоидально для тестирования
      static float phase = 0;
      phase += dt * 0.5;
      simVerticalSpeed = 3.0 * sin(phase);
      
      simAltitude += simVerticalSpeed * dt;
      altitude = simAltitude;
      verticalSpeed = simVerticalSpeed;
      
      // Имитация давления
      pressure = PRESSURE_SEA_LEVEL * exp(-altitude / 8430);
      temperature = 20.0;
      
    } else {
      // ===== РЕЖИМ РЕАЛЬНЫХ ИЗМЕРЕНИЙ =====
      char status;
      double T, P;
      
      status = bmp180.startTemperature();
      if (status != 0) {
        delay(status);
        status = bmp180.getTemperature(T);
        if (status != 0) {
          status = bmp180.startPressure(3);
          if (status != 0) {
            delay(status);
            status = bmp180.getPressure(P, T);
            if (status != 0) {
              pressure = P;
              temperature = T;
              
              // Расчет абсолютной высоты
              float rawAlt = bmp180.altitude(P, PRESSURE_SEA_LEVEL);
              
              // Фильтр Калмана
              float filteredAlt = kalmanUpdate(rawAlt);
              
              // Относительная высота (с учетом baseline)
              float baselineAlt = bmp180.altitude(baselinePressure, PRESSURE_SEA_LEVEL);
              altitude = filteredAlt - baselineAlt;
              
              // Расчет вертикальной скорости
              verticalSpeed = calculateVerticalSpeed(filteredAlt, now);
            }
          }
        }
      }
    }
    
    // Генерация звука
    generateVarioSound(verticalSpeed);
  }
  
  // Передача данных
  if (now - lastTransmitTime >= TRANSMIT_INTERVAL) {
    lastTransmitTime = now;
    
    // Чтение уровня батареи (пример)
    float battery = 4.2; // В реальности нужно читать через analogRead
    
    // Формирование LK8EX1
    String lk8 = buildLK8EX1(pressure, altitude, verticalSpeed, temperature, battery);
    
    // Отправка по Serial
    Serial.print(lk8);
    
    // Отправка по BLE
    if (BLE.connected()) {
      txChar.writeValue((const uint8_t*)lk8.c_str(), lk8.length());
    }
    
    // Отладочный вывод
    static unsigned long lastDebug = 0;
    if (now - lastDebug > 1000) {
      lastDebug = now;
      Serial.print("Alt: ");
      Serial.print(altitude, 2);
      Serial.print(" m, Vario: ");
      Serial.print(verticalSpeed, 2);
      Serial.print(" m/s, Sound: ");
      Serial.println(soundEnabled ? "ON" : "OFF");
    }
  }
}

// ==================== ФУНКЦИЯ ДЛЯ OTA ====================
/*
 * Для поддержки OTA обновлений через смартфон необходимо:
 * 
 * 1. Использовать плату с поддержкой DFU (например, Seeed XIAO nRF52840,
 *    Adafruit Feather nRF52840)
 * 
 * 2. Загрузить соответствующий bootloader (UF2 или MCUboot)
 * 
 * 3. Для инициации OTA можно использовать:
 *    - Двойной сброс (два нажатия Reset в течение 500 мс)
 *    - Специальную BLE-команду (Buttonless DFU)
 * 
 * 4. На смартфоне использовать приложение nRF Connect или Device Firmware Update
 * 
 * Пример кода для Buttonless DFU (требуется Nordic SDK):
 * 
 * #include <bluefruit.h>
 * void startDFU() {
 *   // Отправка команды на переход в режим DFU
 *   // Требуется реализация через Nordic Secure DFU Service
 * }
 * 
 * Подробнее: 
 * - https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/libraries/Adafruit_TinyUSB_Arduino
 * - https://docs.nordicsemi.com/bundle/nrf-connect-sdk/page/nrf/device_guides/working_with_nrf/nrf52/nrf52_dfu.html
 */