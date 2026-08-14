/*
 * Вариометр на базе nRF52840 + DPS310 + пьезоизлучатель
 * Аналог Brauniger IQ One с тональными прерывистыми сигналами,
 * фильтром Калмана, передачей данных по Serial и BLE,
 * поддержкой кнопки (долгое нажатие – сброс высоты, двойное – отключение звука),
 * режимом симуляции и OTA-заглушкой.
 */

#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <ArduinoBLE.h>

// Пины
#define BUTTON_PIN  4   // кнопка (INPUT_PULLUP)
#define BUZZER_PIN  3   // пьезоизлучатель (PWM)

// Константы
#define SERIAL_BAUD 115200
#define ALTITUDE_REF_PRESSURE 1013.25f   // опорное давление для абсолютной высоты (мбар), будет скорректировано при калибровке
#define GRAVITY 9.80665f
#define GAS_CONST 287.05f
#define LAPSE_RATE 0.0065f
#define SEA_LEVEL_PRESSURE 1013.25f

// Параметры звука
#define TONE_FREQ 1000        // частота тона, Гц
#define BASE_TONE_DUR 400     // базовая длительность звука (мс) при нулевой скорости
#define BASE_PAUSE_DUR 400    // базовая длительность паузы (мс) при нулевой скорости
#define MIN_TONE_DUR 50       // минимальная длительность звука
#define MAX_TONE_DUR 800      // максимальная длительность звука
#define MIN_PAUSE_DUR 50
#define MAX_PAUSE_DUR 800
#define SPEED_FACTOR 2.0f     // коэффициент влияния скорости на длительности

// Пороги
#define VARIO_THRESHOLD 0.15f // м/с, ниже которого звук не генерируется

// BLE UART сервис (Nordic UART)
#define BLE_UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_CHAR_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Объекты
Adafruit_DPS310 dps;
BLEService uartService(BLE_UART_SERVICE_UUID);
BLECharacteristic txChar(BLE_TX_CHAR_UUID, BLERead | BLENotify, 20);

// Состояние фильтра Калмана
class KalmanFilter {
public:
  float x[2];   // состояние: [высота, вертикальная скорость]
  float P[2][2]; // ковариационная матрица
  float Q[2][2]; // шум процесса
  float R;       // шум измерения
  float dt;      // шаг по времени

  KalmanFilter() {
    // Инициализация
    x[0] = 0.0f;
    x[1] = 0.0f;
    P[0][0] = 10.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f;  P[1][1] = 10.0f;
    Q[0][0] = 0.01f; Q[0][1] = 0.0f;
    Q[1][0] = 0.0f;  Q[1][1] = 0.1f;
    R = 0.5f;        // шум измерения высоты (м^2)
    dt = 0.1f;
  }

  void predict(float dt) {
    // Матрица перехода F = [[1, dt], [0, 1]]
    float F[2][2] = {{1.0f, dt}, {0.0f, 1.0f}};
    // Предсказание состояния
    float newX[2];
    newX[0] = F[0][0]*x[0] + F[0][1]*x[1];
    newX[1] = F[1][0]*x[0] + F[1][1]*x[1];
    x[0] = newX[0];
    x[1] = newX[1];

    // Предсказание ковариации: P = F * P * F^T + Q
    float P_temp[2][2];
    P_temp[0][0] = F[0][0]*P[0][0] + F[0][1]*P[1][0];
    P_temp[0][1] = F[0][0]*P[0][1] + F[0][1]*P[1][1];
    P_temp[1][0] = F[1][0]*P[0][0] + F[1][1]*P[1][0];
    P_temp[1][1] = F[1][0]*P[0][1] + F[1][1]*P[1][1];

    P[0][0] = P_temp[0][0]*F[0][0] + P_temp[0][1]*F[1][0] + Q[0][0];
    P[0][1] = P_temp[0][0]*F[0][1] + P_temp[0][1]*F[1][1] + Q[0][1];
    P[1][0] = P_temp[1][0]*F[0][0] + P_temp[1][1]*F[1][0] + Q[1][0];
    P[1][1] = P_temp[1][0]*F[0][1] + P_temp[1][1]*F[1][1] + Q[1][1];
  }

  void update(float z) {
    // H = [1, 0]
    float S = P[0][0] + R;
    float K[2] = {P[0][0]/S, P[1][0]/S};
    // Обновление состояния
    float y = z - x[0];
    x[0] += K[0] * y;
    x[1] += K[1] * y;
    // Обновление ковариации
    float P00 = P[0][0] - K[0]*P[0][0];
    float P01 = P[0][1] - K[0]*P[0][1];
    float P10 = P[1][0] - K[1]*P[0][0];
    float P11 = P[1][1] - K[1]*P[0][1];
    P[0][0] = P00; P[0][1] = P01;
    P[1][0] = P10; P[1][1] = P11;
  }
};

// Глобальные переменные
KalmanFilter kalman;
float referencePressure = 1013.25f;  // опорное давление для относительной высоты (мбар)
float relativeAltitude = 0.0f;       // относительная высота, м
float verticalSpeed = 0.0f;          // вертикальная скорость, м/с (положительная - подъем)
bool soundEnabled = true;
bool simulationMode = false;
float simSpeed = 0.0f;               // задаваемая скорость в симуляции
float simAltitude = 0.0f;            // интегрированная высота в симуляции
unsigned long lastSimTime = 0;

// Параметры звука
unsigned long toneDuration = BASE_TONE_DUR;
unsigned long pauseDuration = BASE_PAUSE_DUR;
unsigned long lastSoundToggle = 0;
bool soundOn = false;
bool isMuted = false;                // отключение звука двойным нажатием

// Для обработки кнопки
bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
unsigned long lastButtonReleaseTime = 0;
int clickCount = 0;
bool longPressHandled = false;

// Время последней отправки данных
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 200; // 5 Гц

// Прототипы функций
float readAltitude();
void updateKalman(float h, float dt);
void generateSound(float speed);
void sendData(float alt, float speed);
void handleButton();
void processSerialCommands();
void startSimulation(bool on);
void setSimSpeed(float speed);
void resetAltitude();

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);

  // Инициализация DPS310
  Wire.begin();
  if (!dps.begin_I2C()) {
    Serial.println("DPS310 не найден! Проверьте подключение.");
    while (1) delay(10);
  }
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  dps.enablePressure(true);
  dps.enableTemperature(true);

  // Инициализация BLE
  if (!BLE.begin()) {
    Serial.println("BLE не инициализирован!");
    while (1) delay(10);
  }
  BLE.setDeviceName("Variometer");
  BLE.setLocalName("Variometer");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(txChar);
  BLE.addService(uartService);
  txChar.writeValue("");  // инициализация
  BLE.advertise();
  Serial.println("BLE реклама запущена.");

  // Настройка пинов
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // Калибровка начальной высоты
  float initialAlt = readAltitude();
  referencePressure = SEA_LEVEL_PRESSURE; // мы будем использовать относительную высоту, поэтому опорное давление не нужно
  // Но для вычисления относительной высоты нам нужно запомнить начальное давление
  // Поэтому мы сохраним текущее давление как опорное, а высоту будем считать как изменение давления
  // Проще: при первом чтении зададим referencePressure как текущее давление.
  // Для этого читаем давление и сохраняем.
  float p0 = 0;
  if (dps.readPressure(p0)) {
    referencePressure = p0; // опорное давление для относительной высоты
    Serial.print("Опорное давление установлено: "); Serial.println(referencePressure);
  } else {
    referencePressure = 1013.25f;
  }

  // Инициализация фильтра Калмана
  kalman.x[0] = 0.0f;   // относительная высота 0
  kalman.x[1] = 0.0f;
  kalman.dt = 0.1f;

  // Начальные параметры звука
  toneDuration = BASE_TONE_DUR;
  pauseDuration = BASE_PAUSE_DUR;
  lastSoundToggle = millis();

  Serial.println("Вариометр готов.");
  Serial.println("Команды: sim on/off, simspeed <м/с>, mute, reset, help");
}

void loop() {
  // Чтение давления и вычисление высоты
  float altitude = readAltitude(); // относительная высота в метрах

  // Время цикла
  static unsigned long lastLoopTime = millis();
  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0f;
  if (dt > 0.5f) dt = 0.1f; // ограничим слишком большие dt
  lastLoopTime = now;

  // Если режим симуляции выключен, используем реальные данные
  if (!simulationMode) {
    // Фильтр Калмана
    updateKalman(altitude, dt);
    verticalSpeed = kalman.x[1];
  } else {
    // Симуляция: скорость задана, высота интегрируется
    if (lastSimTime == 0) lastSimTime = now;
    float dtSim = (now - lastSimTime) / 1000.0f;
    lastSimTime = now;
    simAltitude += simSpeed * dtSim;
    verticalSpeed = simSpeed;
    // Для отображения высоты используем simAltitude
    altitude = simAltitude;
    // Также обновляем фильтр для согласованности, но можно не обновлять
    // Мы просто переопределим скорость
  }

  // Генерация звука
  generateSound(verticalSpeed);

  // Отправка данных по Serial и BLE
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    sendData(altitude, verticalSpeed);
  }

  // Обработка кнопки
  handleButton();

  // Обработка команд Serial
  processSerialCommands();

  // Небольшая задержка для стабильности
  delay(10);
}

// Функция чтения давления и пересчета в относительную высоту
float readAltitude() {
  float pressure = 0;
  if (!dps.readPressure(pressure)) {
    return kalman.x[0]; // если не удалось прочитать, возвращаем последнее значение
  }
  // Пересчет в высоту относительно опорного давления (барометрическая формула)
  // h = (T0 / L) * (1 - (P/P0)^(R*L/g))
  // Упрощенно: h = 44330 * (1 - (P/P0)^(1/5.255))
  float ratio = pressure / referencePressure;
  float alt = 44330.0f * (1.0f - pow(ratio, 1.0f/5.255f));
  return alt;
}

// Обновление фильтра Калмана
void updateKalman(float h, float dt) {
  kalman.dt = dt;
  kalman.predict(dt);
  kalman.update(h);
}

// Генерация звука в зависимости от скорости
void generateSound(float speed) {
  if (isMuted) {
    if (soundOn) {
      noTone(BUZZER_PIN);
      soundOn = false;
    }
    return;
  }

  // Если скорость по модулю меньше порога, выключаем звук
  if (fabs(speed) < VARIO_THRESHOLD) {
    if (soundOn) {
      noTone(BUZZER_PIN);
      soundOn = false;
    }
    return;
  }

  // Вычисляем масштабный коэффициент для длительностей
  // При подъеме (speed > 0) длительности уменьшаются, при спуске (speed < 0) - увеличиваются
  float factor = 1.0f - speed * SPEED_FACTOR; // если speed положительна, factor < 1
  // Ограничим factor
  if (factor < 0.1f) factor = 0.1f;
  if (factor > 2.0f) factor = 2.0f;

  // Рассчитываем длительности
  unsigned long newTone = (unsigned long)(BASE_TONE_DUR * factor);
  unsigned long newPause = (unsigned long)(BASE_PAUSE_DUR * factor);
  if (newTone < MIN_TONE_DUR) newTone = MIN_TONE_DUR;
  if (newTone > MAX_TONE_DUR) newTone = MAX_TONE_DUR;
  if (newPause < MIN_PAUSE_DUR) newPause = MIN_PAUSE_DUR;
  if (newPause > MAX_PAUSE_DUR) newPause = MAX_PAUSE_DUR;

  // Обновляем только если изменились параметры
  if (newTone != toneDuration || newPause != pauseDuration) {
    toneDuration = newTone;
    pauseDuration = newPause;
    // Сбросим таймер, чтобы не нарушить текущий цикл
    lastSoundToggle = millis();
    soundOn = false; // принудительно выключим, чтобы перезапустить с новыми параметрами
    noTone(BUZZER_PIN);
  }

  // Управление переключением звука
  unsigned long now = millis();
  unsigned long elapsed = now - lastSoundToggle;

  if (!soundOn) {
    // Если звук выключен, включаем его на время toneDuration
    if (elapsed >= pauseDuration) {
      tone(BUZZER_PIN, TONE_FREQ);
      soundOn = true;
      lastSoundToggle = now;
    }
  } else {
    // Если звук включен, выключаем через toneDuration
    if (elapsed >= toneDuration) {
      noTone(BUZZER_PIN);
      soundOn = false;
      lastSoundToggle = now;
    }
  }
}

// Отправка данных по Serial и BLE с контрольной суммой NMEA
void sendData(float alt, float speed) {
  char buffer[64];
  // Формат: $VARIO,altitude,speed*checksum\r\n
  // Контрольная сумма: XOR всех символов между $ и *
  int len = snprintf(buffer, sizeof(buffer), "$VARIO,%.1f,%.2f", alt, speed);
  // Вычисляем XOR
  uint8_t cs = 0;
  for (int i = 1; i < len; i++) { // начинаем с символа после '$'
    cs ^= buffer[i];
  }
  char out[80];
  snprintf(out, sizeof(out), "%s*%02X\r\n", buffer, cs);

  // Отправка по Serial
  Serial.print(out);

  // Отправка по BLE, если подключено
  if (BLE.connected()) {
    txChar.writeValue((uint8_t*)out, strlen(out));
  }
}

// Обработка кнопки (долгое нажатие 2 сек - сброс, двойное - mute)
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  if (reading == LOW && lastButtonState == HIGH) {
    // Нажатие
    buttonPressTime = now;
    longPressHandled = false;
    lastButtonState = LOW;
  } else if (reading == HIGH && lastButtonState == LOW) {
    // Отпускание
    unsigned long pressDuration = now - buttonPressTime;
    if (pressDuration >= 2000 && !longPressHandled) {
      // Долгое нажатие (>2 сек)
      resetAltitude();
      longPressHandled = true;
      Serial.println("Высота сброшена.");
    } else if (pressDuration < 500) {
      // Короткое нажатие
      // Считаем клики для двойного
      if (now - lastButtonReleaseTime < 500) {
        clickCount++;
      } else {
        clickCount = 1;
      }
      lastButtonReleaseTime = now;
      if (clickCount == 2) {
        // Двойное нажатие - переключение звука
        isMuted = !isMuted;
        if (isMuted) {
          Serial.println("Звук отключен (mute)");
          noTone(BUZZER_PIN);
          soundOn = false;
        } else {
          Serial.println("Звук включен");
        }
        clickCount = 0;
      }
    }
    lastButtonState = HIGH;
  }
  // Сброс счетчика кликов, если прошло более 500 мс после последнего отпускания
  if (clickCount > 0 && (now - lastButtonReleaseTime > 500)) {
    clickCount = 0;
  }
}

// Сброс относительной высоты (установка текущего давления как опорного)
void resetAltitude() {
  float pressure = 0;
  if (dps.readPressure(pressure)) {
    referencePressure = pressure;
    // Обнуляем фильтр Калмана
    kalman.x[0] = 0.0f;
    kalman.x[1] = 0.0f;
    // Если симуляция, то тоже сбрасываем
    if (simulationMode) {
      simAltitude = 0.0f;
    }
    Serial.print("Опорное давление обновлено: "); Serial.println(referencePressure);
  } else {
    Serial.println("Не удалось считать давление для сброса.");
  }
}

// Обработка команд по Serial
void processSerialCommands() {
  if (Serial.available() <= 0) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.startsWith("sim")) {
    if (cmd == "sim on") {
      startSimulation(true);
      Serial.println("Режим симуляции включен.");
    } else if (cmd == "sim off") {
      startSimulation(false);
      Serial.println("Режим симуляции выключен.");
    } else {
      Serial.println("Используйте: sim on / sim off");
    }
  } else if (cmd.startsWith("simspeed")) {
    // simspeed <значение>
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx != -1) {
      String val = cmd.substring(spaceIdx + 1);
      float sp = val.toFloat();
      setSimSpeed(sp);
      Serial.print("Скорость симуляции установлена: "); Serial.println(sp);
    } else {
      Serial.println("Формат: simspeed <м/с>");
    }
  } else if (cmd == "mute") {
    isMuted = !isMuted;
    if (isMuted) {
      Serial.println("Звук отключен (mute)");
      noTone(BUZZER_PIN);
      soundOn = false;
    } else {
      Serial.println("Звук включен");
    }
  } else if (cmd == "reset") {
    resetAltitude();
    Serial.println("Высота сброшена.");
  } else if (cmd == "help") {
    Serial.println("Доступные команды:");
    Serial.println("  sim on / off - включить/выключить симуляцию");
    Serial.println("  simspeed <м/с> - установить скорость в симуляции");
    Serial.println("  mute - переключить звук (вкл/выкл)");
    Serial.println("  reset - сбросить высоту (аналог долгого нажатия)");
    Serial.println("  help - эта справка");
  } else {
    Serial.println("Неизвестная команда. Введите help для списка.");
  }
}

// Включение/выключение режима симуляции
void startSimulation(bool on) {
  simulationMode = on;
  if (on) {
    lastSimTime = millis();
    simAltitude = kalman.x[0]; // начинаем с текущей высоты
  } else {
    // При выходе из симуляции используем последние данные фильтра
    simAltitude = 0.0f;
    simSpeed = 0.0f;
  }
}

void setSimSpeed(float speed) {
  if (simulationMode) {
    simSpeed = speed;
  } else {
    Serial.println("Сначала включите режим симуляции: sim on");
  }
}

// Заглушка для OTA (обновление по воздуху)
// В реальном проекте здесь можно реализовать DFU через BLE,
// но для текущего скетча оставляем пустую функцию.
void handleOTA() {
  // Не реализовано в данном демо.
}