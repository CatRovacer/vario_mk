#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DPS310.h>  // Поддерживает SPL06-001 и DPS310
#include <KalmanFilter.h>      // Библиотека фильтрации
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ================= Конфигурация звука (Эмуляция Brauniger IQ-ONE) =================
// Параметры тональности (частота = базовая + скорость * коэффициент)
const float BASE_FREQ = 800.0;    // Базовый тон (Гц) при нулевой скорости (нейтральный сигнал)
const float CLIMB_FACTOR = 120.0; // Крутизна роста тона (Гц / (м/с)) при подъеме
const float SINK_FACTOR = 100.0;  // Крутизна спада тона при спуске (будет пищать ниже 800Гц)
const float MIN_FREQ = 300.0;     // Нижняя граница слышимости
const float MAX_FREQ = 3000.0;    // Верхняя граница для защиты динамика

// Звуковой порог (порог чувствительности вариометра, "A-Int" в IQ-ONE)
// При скорости |vario| < DEADBAND звук не издается или имеет неизменный тон (стоим на месте)
const float DEADBAND = 0.1; // м/с

// Длительность импульсов (имитация прерывистого звука при сильном падении, как в IQ-ONE)
// В браунигере при сильном снижении тон меняется на прерывистый.
// Здесь: если скорость снижения > SINK_ALARM_THRESH, включаем режим бипера (прерывистый тон)
const float SINK_ALARM_THRESH = -1.5; // м/с (ниже -1.5 м/с)
const int BEEP_INTERVAL_MS = 400;     // Длительность импульса и паузы (мс)

// ================= Аппаратные пины =================
#define SDA_PIN 12  // 8 
#define SCL_PIN 13  // 9

#define PIN_BUZZER 15      // Пин пьезодинамика (PWM)
#define PIN_CALIB_BTN 0    // Кнопка калибровки (замыкает на GND)

// ================= Глобальные объекты =================
Adafruit_DPS310 dps;
KalmanFilter kf;            // Фильтр Калмана (сглаживание резких скачков)

// Переменные состояния
float currentVerticalSpeed = 0.0; // Текущая вертикальная скорость (м/с)
float referencePressure = 0.0;     // Опорное давление на нулевой высоте
float currentAltitude = 0.0;       // Высота над уровнем моря
float previousAltitude = 0.0;
unsigned long lastAltitudeTime = 0;

// Для Bluetooth
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;

// Таймеры для звука
unsigned long lastBeepToggle = 0;
bool beepState = false;

// Настройки звукового режима
bool isAlarmBeeping = false;

// ================= Объявление класса Callback для BLE =================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client BLE connected!");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client BLE disconnected. Restarting advertising...");
    pServer->startAdvertising(); // Перезапускаем рекламу, чтобы можно было переподключиться
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.print("BLE RX: ");
      Serial.println(rxValue.c_str());
      // Здесь можно обрабатывать команды с телефона (например, "CAL" для калибровки)
      if (rxValue == "CAL") {
        calibrateZero();
      }
    }
  }
};

// ================= Функции =================
void calibrateZero() {
  if (dps.getPressure(&referencePressure)) {
    Serial.print("Zero calibration set at pressure: ");
    Serial.println(referencePressure);
    // Дополнительно можно подать сигнал подтверждения
    tone(PIN_BUZZER, 2000, 200);
  } else {
    Serial.println("Calibration failed: can't read pressure!");
  }
}

float getVerticalSpeed() {
  if (!dps.getPressure(&referencePressure)) { // В реальности нужно читать давление каждую итерацию
    return currentVerticalSpeed; // Возвращаем старое значение при ошибке
  }
  
  // Получаем текущую высоту по барометрической формуле (упрощенно)
  // P = P0 * exp(-H/8500) -> H = 8500 * ln(P0/P)
  float newAltitude = 8500.0 * log(referencePressure / currentAltitude); // Это некорректно, используем правильную формулу в setup
  // Исправлено: лучше использовать встроенную функцию altimeter
  // DPS310 умеет считать высоту сама, но мы используем давление для ручного дифференцирования.
  
  // --- АЛЬТЕРНАТИВНЫЙ ПРАВИЛЬНЫЙ МЕТОД (без готовой altimeter) ---
  // Перерасчет высоты через барометрическую формулу:
  // H = (T0 / L) * ( (P0/P)^(L*R/g) - 1 ) — сложно.
  // Проще использовать показания датчика, если он поддерживает altimeter.
  
  float pressurePa;
  if (!dps.getPressure(&pressurePa)) return currentVerticalSpeed;
  
  // Стандартная барометрическая формула (упрощенная, без учета температуры)
  // H = 44330 * (1 - (P/P0)^(1/5.255))
  // Где P0 = 1013.25 hPa (стандартное давление) или referencePressure при калибровке.
  float P0_hPa = (referencePressure > 0) ? referencePressure : 1013.25;
  float P_hPa = pressurePa;
  
  float H = 44330.0 * (1.0 - pow(P_hPa / P0_hPa, 1.0/5.255));
  
  // Обновляем время
  unsigned long now = millis();
  float dt = (now - lastAltitudeTime) / 1000.0;
  if (dt < 0.001) dt = 0.1;
  
  // Вычисляем вертикальную скорость (дифференциал высоты)
  float rawSpeed = (H - currentAltitude) / dt;
  
  // Обновляем высоту
  currentAltitude = H;
  lastAltitudeTime = now;
  
  // Применяем фильтр Калмана для сглаживания
  // (kf.update(rawSpeed) — если у вас библиотека с методом update)
  // Простая реализация фильтра Калмана первого порядка:
  static float err_measure = 0.5;   // Шум измерений (м/с)
  static float err_estimate = 0.2;
  static float kalman_gain = 0;
  static float current_estimate = 0;
  
  kalman_gain = err_estimate / (err_estimate + err_measure);
  current_estimate = current_estimate + kalman_gain * (rawSpeed - current_estimate);
  err_estimate = (1 - kalman_gain) * err_estimate + fabs(current_estimate) * 0.05; // Адаптивный шум
  
  return current_estimate;
}

void updateAudio(float vario_speed) {
  unsigned long now = millis();
  bool shouldBeep = false;
  int freq = BASE_FREQ;
  
  // Проверка на зону молчания (Deadband)
  if (abs(vario_speed) < DEADBAND) {
    noTone(PIN_BUZZER);
    return;
  }
  
  // Расчет частоты (тональности) в зависимости от подъема или спуска
  if (vario_speed >= 0) {
    // Подъем: частота растет линейно
    freq = BASE_FREQ + (vario_speed * CLIMB_FACTOR);
    freq = constrain(freq, (int)BASE_FREQ, (int)MAX_FREQ);
    shouldBeep = true; // Непрерывный сигнал
    isAlarmBeeping = false;
  } 
  else {
    // Спуск: частота падает линейно. Чем быстрее падаем, тем ниже тон.
    // Но если падение очень сильное (ниже SINK_ALARM_THRESH), включаем "панический" режим (прерывистый).
    float abs_sink = abs(vario_speed);
    freq = BASE_FREQ - (abs_sink * SINK_FACTOR);
    freq = constrain(freq, (int)MIN_FREQ, (int)BASE_FREQ);
    
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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CALIB_BTN, INPUT_PULLUP);
  
  // 1. Инициализация датчика давления
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!dps.begin_I2C()) {
    Serial.println("DPS310/SPL06 not found! Check wiring.");
    while (1) delay(100);
  }
  dps.setPressureOversampling(7);  // Максимальное усреднение для стабильности
  dps.setMode(DPS310_MODE_CONTINUOUS);
  
  // Первичная калибровка (сбрасываем высоту на текущую)
  delay(500);
  calibrateZero();
  currentAltitude = 0; // Относительная высота
  previousAltitude = 0;
  lastAltitudeTime = millis();
  
  // 2. Инициализация BLE (BLE SPP Server)
  BLEDevice::init("ESP32_Vario_Brauniger");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Создаем сервис с UUID, похожим на классический UART сервис Nordic
  BLEService *pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  
  // Характеристика для отправки данных (Notify)
  pTxCharacteristic = pService->createCharacteristic(
                      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  // Характеристика для приема команд (Write)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE UART Service ready. Connect with Serial Bluetooth Terminal.");
  
  // Настройки фильтра Калмана (инициализация)
  // Для простоты используется статический фильтр в getVerticalSpeed()
}

void loop() {
  // 1. Обработка кнопки калибровки (нажатие)
  static bool lastBtnState = HIGH;
  bool btnState = digitalRead(PIN_CALIB_BTN);
  if (lastBtnState == HIGH && btnState == LOW) {
    calibrateZero();
    delay(50); // Антидребезг
  }
  lastBtnState = btnState;
  
  // 2. Чтение датчика и расчет вертикальной скорости
  // Для стабильности обновляем вариометр не чаще 20 Гц (50мс)
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead >= 50) {
    currentVerticalSpeed = getVerticalSpeed();
    lastSensorRead = millis();
    
    // Логирование в Serial
    Serial.print("Vario: ");
    Serial.print(currentVerticalSpeed, 2);
    Serial.print(" m/s   Press: ");
    // В реальности нужно хранить текущее давление глобально, здесь упрощенно
    float p;
    if(dps.getPressure(&p)) Serial.println(p);
    else Serial.println("---");
    
    // 3. Отправка данных по BLE (как Serial Port)
    if (deviceConnected) {
      char buffer[50];
      snprintf(buffer, sizeof(buffer), "VS=%.2f,Alt=%.1f\r\n", currentVerticalSpeed, currentAltitude);
      pTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
      pTxCharacteristic->notify();
    }
    
    // 4. Обновление звука
    updateAudio(currentVerticalSpeed);
  }
  
  // Небольшая задержка для стабильности цикла (не блокирует)
  delay(10);
}