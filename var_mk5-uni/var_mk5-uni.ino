/*
   Вариометр на nRF52840 + DPS310/LPS22HB + пьезоизлучатель
   Формируем строку в прибор: $LK8EX1,altitude,vspeed*CS
*/
#define ADAFRUIT_DPS310_LIB false
#define DPS310_LIB      false
#define LPS22HB_LIB	false

#if ADAFRUIT_DPS310_LIB
#include <Adafruit_DPS310.h>
#elif DPS310_LIB
#include <DPS310.h>
#elif LPS22HB_LIB
#include <Arduino_LPS22HB.h>
#endif

#define DPS310_EN 0
#define LPS22HB_EN 0
#define SPL06_EN 0
#define BMP280_EN 1

#define BLE_EN 0

#define VERBOSE_ENABLED true
#include "def_dbg-print.h"
#if 0
#ifdef VERBOSE_ENABLED
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif
#endif


// Параметры симуляции
#define SIMULATION_MODE false  // true - режим симуляции, false - реальные данные

#include <Wire.h>
#if BLE_EN
  #include <ArduinoBLE.h>
#endif

#if DPS310_EN
  #include <Adafruit_DPS310.h>
#endif

#if LPS22HB_EN
  #include <Arduino_LPS22HB.h>
#endif

#if SPL06_EN
  #include <SPL06-001.h>
#endif

#if BMP280_EN
  #include <Adafruit_BMP280.h>
#endif

// ------------------- Настройки пинов -------------------
#define BUTTON_PIN      2   // 7 кнопка (активный низкий уровень с подтяжкой)
#define BUZZER_PIN      6   // PWM выход на пьезоизлучатель
#define LED_PIN         13  // встроенный светодиод для индикации

//esp32c6
#define SDA_PIN 20
#define SCL_PIN 19

// ------------------- Параметры вариометра -------------------
#define SERIAL_BAUD 115200
#define ALTITUDE_OFFSET 0.0    // начальное смещение высоты (обнуляется кнопкой)
#define PRESSURE_SEA_LEVEL 1013.25 // стандартное давление на уровне моря (гПа)

// Параметры фильтра Калмана (одномерный для вертикальной скорости)
#define KALMAN_Q 0.01   // шум процесса (скорость изменения скорости)
#define KALMAN_R 0.5    // шум измерений (шум датчика)

// Параметры звукового сигнала
#define TONE_FREQUENCY 800    // частота тона (Гц)
#define MIN_PULSE_DURATION 50   // минимальная длительность звука (мс)
#define MAX_PULSE_DURATION 500  // максимальная длительность звука (мс)
#define MIN_PAUSE_DURATION 50  // минимальная пауза (мс)
#define MAX_PAUSE_DURATION 2000 // максимальная пауза (мс)
#define SPEED_THRESHOLD 0.1    // порог скорости для включения звука (м/с)

// Параметры кнопки
#define LONG_PRESS_MS 2000     // длительное нажатие (мс)
#define DOUBLE_CLICK_MAX_MS 300 // макс интервал между нажатиями для двойного клика

// ------------------- Глобальные объекты -------------------
#if DPS310_EN
  Adafruit_DPS310 BARO;
#endif

#if LPS22HB_EN
#endif

#if SPL06_EN
  SPL06 BARO; 
#endif

#if BMP280_EN
  Adafruit_BMP280 BARO; 
#endif

#if BLE_EN
  BLEService uartService(BLE_UART_SERVICE_UUID);
  BLECharacteristic txChar(BLE_TX_CHAR_UUID, BLERead | BLENotify, 20);

  BLEService varioService("19B10000-E8F2-537E-4F6C-D104768A1214");
  BLEFloatCharacteristic altitudeChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
  BLEFloatCharacteristic varioChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
  BLEIntCharacteristic audioStateChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
#endif

const char *devTitle = "VgtVarik-MK5";
const char *NMEA_PROMPT = "$LK8EX1";

// ------------------- Переменные состояния -------------------
float altitude = 0.0;          // текущая высота (м)
float verticalSpeed = 0.0;     // вертикальная скорость (м/с)
float altitudeOffset = 0.0;    // смещение высоты (обнуляемое)
float lastAltitude = 0.0;
unsigned long lastAltTime = 0;

// Фильтр Калмана
float kalmanX = 0.0;     // состояние: скорость
float kalmanP = 1.0;     // ковариация

// Звук
bool soundEnabled = true;
bool isBeeping = false;
unsigned long beepStartTime = 0;
unsigned long beepDuration = 0;
unsigned long pauseDuration = 0;
bool isPausing = false;

// Кнопка
unsigned long buttonPressTime = 0;
bool buttonWasPressed = false;
int clickCount = 0;
unsigned long lastClickTime = 0;

// Таймеры
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_INTERVAL = 50; // мс (20 Гц)

// Режим симуляции
float simTime = 0.0;

#if 1
float currentAltitude = 0.0;         // Current altitude (m)
float referencePressure = 1013.25;
#endif

// ------------------- Прототипы функций -------------------
#if BLE_EN
  void setupBLE();
  void startOTAUpdate();
#endif
int HardwareInit();
void readSensor();
void updateVario();
void kalmanFilter(float measurement);
void generateSound();
void handleButton();
void sendNMEAData(int alt, int vspeed);
void simulateVario();

void CalibrateAltitude();
float CalculateAltitude(float press);


// ------------------- setup() -------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
//  while (!Serial) delay(10);

  // Инициализация пинов
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  if (!HardwareInit()){
  Serial.println("Failed to initialize BARO sensor!"); while(1);
  }
  // Инициализация BLE
#if BLE_EN
  setupBLE();
#endif

  // Начальные значения
  readSensor();

//  CalibrateAltitude();

/////////
  altitudeOffset = altitude;
  readSensor();
  lastAltitude = altitude;
  lastAltTime = millis();
  kalmanX = 0.0;

  Serial.print("Variometer started; Last alt (cm):"); Serial.println(lastAltitude*100);
}

// ------------------- loop() -------------------
void loop() {
  // Обработка кнопки
  handleButton();

  // Чтение датчика с заданной частотой
  if (millis() - lastSensorReadTime >= SENSOR_INTERVAL) {
    lastSensorReadTime = millis();
    if (!SIMULATION_MODE) {
      readSensor();
      updateVario();
    } else {
      simulateVario();
    }
    // Фильтр Калмана применяется внутри updateVario или simulateVario
//    sendNMEAData((int)altitude*100, (int)verticalSpeed*100);
  }

  // Генерация звука
  generateSound();

  // Обработка BLE (поддержка OTA и команд)
#if BLE_EN
  BLE.poll();
#endif
}

// ------------------- Functions implementation -------------------
// Sensor initialize
int HardwareInit(){
if (SIMULATION_MODE) return (0);
  Wire.begin(SDA_PIN, SCL_PIN);

#if DPS310_EN
  if (!BARO.begin_I2C()) {
    Serial.println("Failed to initialize DPS310 BARO sensor!");
    while (1) delay(10);
  }

  BARO.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  BARO.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  //BARO.enablePressure(true);
  //BARO.enableTemperature(true);
#endif

#if LPS22HB_EN
  if (!BARO.begin()) {
    Serial.println("Failed to initialize LPS22HB BARO sensor!");
    while (1) delay(10);
  }
#endif

#if SPL06_EN
  if (!BARO.begin()) {
    Serial.println("Failed to initialize SPL06-01 BARO sensor!");
    while(1) delay(10);
  }
    //following call to setSampling is optional
  BARO.setSampling(SPL06::MODE_BACKGND_BOTH,     /* Operating Mode. */
                  SPL06::SAMPLING_X16,     /* Temperature oversampling */
                  SPL06::SAMPLING_X16,    /* Pressure oversampling */
                  SPL06::RATE_X16,      /* Temprature Rate */
                  SPL06::RATE_X16);   /* Pressure Rate */

#endif

#if BMP280_EN
  if (!BARO.begin()) {
    Serial.println("Failed to initialize BMP280 BARO sensor!"); return (1);
    while(1) delay(10);
    // Настройка BMP280: режим нормальный, oversampling x2, фильтр x4
  BARO.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2, // Температура
                  Adafruit_BMP280::SAMPLING_X2, // Давление
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_125);
  }
#endif
  return(0);
}

//  BLE initialize
#if BLE_EN
void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1) delay(10);
  }

  BLE.setDeviceName("Vario nRF52840");
  BLE.setLocalName(devTitle);

  // Создаём сервис для передачи данных (UART-подобный)
  BLEService varioService("19B10000-E8F2-537E-4F6C-D104768A1214"); // UUID для UART
  varioCharacteristic = BLECharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                                           BLEWrite | BLENotify | BLERead,
                                           20);
  varioDescriptor.setValue("Vario Data");
  varioCharacteristic.addDescriptor(varioDescriptor);

  varioService.addCharacteristic(varioCharacteristic);
  BLE.addService(varioService);

  // Характеристика для управления OTA (может быть использована для активации DFU)
  BLECharacteristic otaCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214",
                                      BLEWrite | BLERead,
                                      1);
  otaCharacteristic.setValue(0);
  BLE.addCharacteristic(otaCharacteristic);

  BLE.advertise();

  Serial.print("Bluetooth active - device name: "); Serial.println(devTitle);
}
#endif

/////////////////////////////////////////////
// Read BARO and Altitude calculation
/////////////////////////////////////////////

void readSensor() {
float pressure;

#if DPS310_EN
  sensors_event_t pressureEvent, tempEvent;
  BARO.getEvents(&pressureEvent, &tempEvent);
  if (pressureEvent.pressure != 0) {
    pressure = pressureEvent.pressure; // в гПа
    // Барометрическая формула для высоты (стандартная атмосфера)
//    altitude = 44330.0 * (1.0 - pow(pressure / PRESSURE_SEA_LEVEL, 0.1903));
//    altitude -= altitudeOffset;
  }
#endif

#if LPS22HB_EN
  if (BARO.readPressure()) {
  pressure = BARO.readPressure(MILLIBAR);
  }
#endif

#if SPL06_EN
  if (BARO.readPressureMBar()) {
  pressure = BARO.readPressureMBar();
  }
#endif

#if BMP280_EN
  if (BARO.readPressure()) {
  pressure = BARO.readPressure();
  }
#endif

    altitude = 44330.0 * (1.0 - pow(pressure / /*referencePressure*/ PRESSURE_SEA_LEVEL, 0.1903));
    altitude -= altitudeOffset;

}

// Обновление вертикальной скорости с применением фильтра Калмана
void updateVario() {
  static unsigned long prevTime = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // секунды
  prevTime = currentTime;

  if (dt <= 0.001) return;

  // Вычисляем сырую скорость по разности высот
  float rawSpeed = (altitude - lastAltitude) / dt;
  lastAltitude = altitude;

  // Применяем фильтр Калмана
  kalmanFilter(rawSpeed);
  verticalSpeed = kalmanX;
}

// Одномерный фильтр Калмана для скорости
void kalmanFilter(float measurement) {
  // Прогноз
  kalmanP += KALMAN_Q;
  // Обновление
  float K = kalmanP / (kalmanP + KALMAN_R);
  kalmanX += K * (measurement - kalmanX);
  kalmanP *= (1.0 - K);
}

// Генерация звуковых сигналов
void generateSound() {
  if (!soundEnabled) {
    // Если звук отключен - убедиться, что пьезо выключен
    if (isBeeping) {
      analogWrite(BUZZER_PIN, 0);
      isBeeping = false;
    }
    return;
  }

  float speed = verticalSpeed;
  float absSpeed = fabs(speed);

  // Если скорость меньше порога - молчание
  if (absSpeed < SPEED_THRESHOLD) {
    if (isBeeping) {
      analogWrite(BUZZER_PIN, 0);
      isBeeping = false;
    }
    isPausing = false;
    return;
  }

  // Расчет длительностей в зависимости от скорости
  // При подъеме (speed > 0) - уменьшаем длительности, при спуске - увеличиваем
  // Используем обратную пропорцию, но ограничиваем
  float factor = absSpeed / 5.0; // предполагаем макс скорость 5 м/с
  if (factor > 1.0) factor = 1.0;
  
  if (speed > 0) { // подъем
    beepDuration = (unsigned long)(MAX_PULSE_DURATION - (MAX_PULSE_DURATION - MIN_PULSE_DURATION) * factor);
    pauseDuration = (unsigned long)(MAX_PAUSE_DURATION - (MAX_PAUSE_DURATION - MIN_PAUSE_DURATION) * factor);
  } else { // спуск
    beepDuration = (unsigned long)(MIN_PULSE_DURATION + (MAX_PULSE_DURATION - MIN_PULSE_DURATION) * factor);
    pauseDuration = (unsigned long)(MIN_PAUSE_DURATION + (MAX_PAUSE_DURATION - MIN_PAUSE_DURATION) * factor);
  }

  // Управление состоянием звука
  unsigned long now = millis();
  if (!isBeeping && !isPausing) {
    // Начинаем новый цикл
    isBeeping = true;
    beepStartTime = now;
    analogWrite(BUZZER_PIN, 128); // включить тон (можно использовать tone, но для PWM используем analogWrite)
    // Для генерации тона через PWM проще использовать библиотеку Tone, но для nRF52840 можно использовать функцию tone().
    // Здесь для упрощения используем tone (если доступна). Если нет, можно реализовать через таймер.
    tone(BUZZER_PIN, TONE_FREQUENCY);
  }

  if (isBeeping && (now - beepStartTime >= beepDuration)) {
    // Заканчиваем звук, начинаем паузу
    analogWrite(BUZZER_PIN, 0);
    noTone(BUZZER_PIN);
    isBeeping = false;
    isPausing = true;
    beepStartTime = now; // переиспользуем для паузы
  }

  if (isPausing && (now - beepStartTime >= pauseDuration)) {
    isPausing = false;
  }
}

// Обработка кнопки (долгое нажатие, двойное)
void handleButton() {
  static bool lastButtonState = HIGH;
  bool currentState = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  // Нажатие (переход от HIGH к LOW)
  if (lastButtonState == HIGH && currentState == LOW) {
    buttonPressTime = now;
    buttonWasPressed = true;
  }

  // Отпускание (переход от LOW к HIGH)
  if (lastButtonState == LOW && currentState == HIGH) {
    if (buttonWasPressed) {
      unsigned long pressDuration = now - buttonPressTime;
      if (pressDuration >= LONG_PRESS_MS) {
        // Долгое нажатие - обнуление высоты
        altitudeOffset = altitude; // теперь текущая высота станет нулевой
        Serial.println("Высота обнулена.");
        // Мигаем светодиодом для подтверждения
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        clickCount = 0; // сброс счетчика кликов
      } else {
        // Короткое нажатие - считаем для двойного клика
        if (now - lastClickTime <= DOUBLE_CLICK_MAX_MS) {
          clickCount++;
        } else {
          clickCount = 1;
        }
        lastClickTime = now;

        if (clickCount >= 2) {
          // Двойное нажатие - переключение звука
          soundEnabled = !soundEnabled;
          Serial.print("Sound: ");
          Serial.println(soundEnabled ? "ON" : "OFF");
          clickCount = 0;
          // Сигнал светодиодом
          digitalWrite(LED_PIN, HIGH);
          delay(50);
          digitalWrite(LED_PIN, LOW);
        }
      }
      buttonWasPressed = false;
    }
  }

  lastButtonState = currentState;
}

// Отправка NMEA данных по Serial и BLE
void sendNMEAData(int h, int v) {
  // Формируем строку: $LK8EX1,altitude,vspeed*checksum
  char buffer[50];
  // float h = altitude; // уже с учетом смещения
  // float v = verticalSpeed;
  // Контрольная сумма (XOR всех байт, кроме $ и *)
  uint8_t checksum = 0;
  sprintf(buffer, "$LK8EX1,%d,%d", (int)h, (int)v);
  // Вычисляем XOR по байтам строки после $
  for (int i = 1; i < strlen(buffer); i++) {
    checksum ^= buffer[i];
  }
  char out[60];
  sprintf(out, "%s*%02X\n", buffer, checksum);

  // Send via regular Serial
  Serial.print(out);
  //  Send via  BLE
#if BLE_EN
  if (BLE.connected()) {
    varioCharacteristic.write((uint8_t*)out, strlen(out));
  }
#endif
}

// Режим симуляции: генерирует изменяющуюся скорость по синусоиде
void simulateVario() {
  simTime += 0.1;
  float simSpeed = 2.0 * sin(simTime * 0.5); // амплитуда 2 м/с, период ~12 с
  // Имитируем изменение высоты (интегрируем скорость)
  static float simAlt = 0.0;
  simAlt += simSpeed * 0.1; // dt=0.1 с
  altitude = simAlt;
  verticalSpeed = simSpeed; // пропускаем фильтр Калмана для наглядности
  // Можно применить фильтр и к симуляции
  kalmanFilter(simSpeed);
//  verticalSpeed = kalmanX; // закомментировать, если хотим сырую скорость
}

#if 0
float CalculateAltitude(float pressure_hPa) {
  // International barometric formula
  // Altitude = 44330 * (1 - (P/P0)^(1/5.255))
  return 44330.0 * (1.0 - pow(pressure_hPa / referencePressure, 0.19029));
}

void CalibrateAltitude() {
  
  // Read multiple samples to get stable reference pressure
  float sumPressure = 0;
  for (int i = 0; i < 50; i++) {
      if (BARO.readPressure()) {
        sumPressure += BARO.readPressure(MILLIBAR);
        }
    delay(10);
  }
  
  referencePressure = sumPressure / 50.0;
  
  // Calculate current altitude from calibrated reference
  float currentPress = BARO.readPressure(MILLIBAR);
  currentAltitude = CalculateAltitude(currentPress);
  
  // Reset Kalman filter with current altitude
//  kalmanFilter.init(currentAltitude, 0.05, 0.3);
  kalmanFilter(currentAltitude);
//  currentAltitude = kalmanX; // закомментировать, если хотим сырую скорость
  
  Serial.print("CalibrateAltitude. Reference pressure set to (hpa): ");
  Serial.println(referencePressure);
  Serial.print("Current altitude (cm): ");
  Serial.println(currentAltitude*100);
  
}

#endif

// Запуск OTA обновления (заглушка, вызывает переход в DFU)
#if BLE_EN
void startOTAUpdate() {
  // Для реального OTA необходимо, чтобы загрузчик поддерживал DFU.
  // На nRF52840 можно использовать функцию для перехода в режим DFU:
  // NVIC_SystemReset(); или использовать библиотеку bootloader.
  // Здесь просто печатаем сообщение.
  Serial.println("Переход в режим OTA...");
  // В реальном коде можно вызвать: bootloader_start() или аналогично.
  // Например, если используется Adafruit nRF52 Bootloader, можно вызвать:
  // if (Bootloader.isValid()) Bootloader.enter();
  // Однако для этого нужно подключить библиотеку.
  // В данном примере мы просто перезагружаемся.
  // NVIC_SystemReset();
}
#endif