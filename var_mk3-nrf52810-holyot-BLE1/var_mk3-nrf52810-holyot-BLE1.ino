/*****************************************************************************
 * Вариометр и альтиметр для Holyiot nRF52810 (Arduino)
 * Датчик: LPS22HB (библиотека Arduino_LPS22HB)
 * Передача данных: Bluetooth Serial (BLE UART)
 * Эмуляция звука Brauniger IQ ONE (без динамика - только данные)
 * Фильтр Калмана (целочисленная реализация)
 * Калибровка по кнопке
 * Совместимость с XCSoar и LK8000
 *****************************************************************************/
#define BLE_EN 0

#include <Arduino.h>
#include <Arduino_LPS22HB.h>
#if BLE_EN
#include <bluefruit.h>
#include <ArduinoBLE.h>
#endif

// ============================================================================
// Конфигурация
// ============================================================================
#define SAMPLE_INTERVAL_MS    50      // 20 Гц采样
#define BUTTON_PIN            7       // Кнопка калибровки (GPIO 7 на Holyiot)
#define LED_PIN               17      // Встроенный светодиод (GPIO 17)

// Параметры фильтра Калмана
#define KALMAN_Q              10      // Шум процесса (в см)
#define KALMAN_R              50      // Шум измерения (в см)
#define KALMAN_INIT_P         1000    // Начальная ковариация

// ============================================================================
// Глобальные переменные
// ============================================================================
//LPS22HB BARO;

#if BLE_EN
  BLESerial bleSerial;  // BLE UART сервис
#endif

// Фильтр Калмана (целочисленный)
struct KalmanFilter {
    int32_t x;        // Состояние: высота (в см)
    int32_t p;        // Ковариация
    int32_t q;        // Шум процесса
    int32_t r;        // Шум измерения
    int32_t k;        // Коэффициент Калмана (умножен на 1000)
    
    void init(int32_t initial_x, int32_t q_val, int32_t r_val, int32_t p_val) {
        x = initial_x;
        p = p_val;
        q = q_val;
        r = r_val;
        k = 0;
    }
    
    int32_t update(int32_t measurement) {
        // Прогноз
        p = p + q;
        
        // Вычисление коэффициента Калмана (масштабированный)
        // k = p / (p + r)
        int32_t divisor = p + r;
        if (divisor == 0) divisor = 1;
        k = (p * 1000) / divisor;  // k * 1000
        
        // Обновление состояния
        // x = x + k * (measurement - x)
        int32_t error = measurement - x;
        int32_t correction = (k * error) / 1000;
        x = x + correction;
        
        // Обновление ковариации
        // p = (1 - k) * p
        p = ((1000 - k) * p) / 1000;
        
        return x;
    }
};

// Структура данных датчика
struct SensorData {
    float pressure;       // Давление в Па
    float temperature;    // Температура в °C
    int32_t altitude;     // Высота в см
    int16_t climb;        // Вертикальная скорость в см/с
};

// Глобальные переменные
static KalmanFilter kalman;
static SensorData sensorData;
static int32_t basePressure = 101325;  // Базовое давление для калибровки
static int32_t prevAltitude = 0;
static uint32_t lastSampleTime = 0;
static int16_t filteredClimb = 0;

// Параметры звуковой эмуляции (без динамика, только логика)
static struct SoundParams {
    bool enabled;
    uint16_t frequency;     // Частота тона (Гц)
    uint16_t period_ms;     // Период прерывистости (мс)
    bool sound_on;
    uint32_t last_toggle;
} sound;

// ============================================================================
// Фильтр Калмана
// ============================================================================
void kalmanInit(int32_t initial_altitude) {
    kalman.init(initial_altitude, KALMAN_Q, KALMAN_R, KALMAN_INIT_P);
}

int32_t kalmanUpdate(int32_t measurement) {
    return kalman.update(measurement);
}

// ============================================================================
// Расчет высоты по давлению (без float)
// ============================================================================
int32_t pressureToAltitude(int32_t pressure, int32_t base_press) {
    // Используем барометрическую формулу с целочисленной аппроксимацией
    // altitude = 44330 * (1 - (pressure/base_pressure)^0.1903)
    // Для целочисленного расчета используем разложение в ряд
    
    if (pressure <= 0 || base_press <= 0) return 0;
    
    // Относительное давление в процентах * 1000
    int32_t ratio = (pressure * 1000) / base_press;
    
    // Аппроксимация (1 - ratio^0.1903) через таблицу
    // Используем кусочно-линейную аппроксимацию
    int32_t factor;
    if (ratio >= 900) {
        factor = (1000 - ratio) * 15 / 100;  // ~1.5% на 1% изменения давления
    } else if (ratio >= 800) {
        factor = (1000 - ratio) * 16 / 100;
    } else if (ratio >= 700) {
        factor = (1000 - ratio) * 17 / 100;
    } else if (ratio >= 600) {
        factor = (1000 - ratio) * 18 / 100;
    } else {
        factor = (1000 - ratio) * 19 / 100;
    }
    
    // Высота в сантиметрах: 44330 * factor / 1000
    int32_t altitude_cm = (44330 * factor) / 1000;
    
    return altitude_cm;
}

// ============================================================================
// Эмуляция звука Brauniger IQ ONE (без динамика)
// ============================================================================
void updateSoundFromClimb(int16_t climb_cm_s) {
    // climb_cm_s: положительный = подъем, отрицательный = снижение
    int16_t absClimb = (climb_cm_s > 0) ? climb_cm_s : -climb_cm_s;
    
    // Нейтральная зона (тишина)
    if (absClimb < 15) {  // менее 15 см/с
        sound.enabled = false;
        return;
    }
    
    if (climb_cm_s > 0) {
        // ПОДЪЕМ: непрерывный тон, частота растет
        // 0.5 м/с = 600 Гц, 5 м/с = 1200 Гц
        int16_t climb_m_s = absClimb / 100;  // в м/с * 10 для точности
        
        if (climb_m_s < 5) climb_m_s = 5;     // минимум 0.5 м/с
        if (climb_m_s > 50) climb_m_s = 50;   // максимум 5 м/с
        
        // Частота: 600 + (speed - 0.5) * 133.3
        sound.frequency = 600 + ((climb_m_s - 5) * 133) / 10;
        if (sound.frequency < 600) sound.frequency = 600;
        if (sound.frequency > 1200) sound.frequency = 1200;
        
        sound.enabled = true;
        sound.sound_on = true;
        sound.period_ms = 0;  // непрерывный
        
    } else {
        // СНИЖЕНИЕ: прерывистый тон, частота падает, период растет
        int16_t climb_m_s = absClimb / 100;  // в м/с * 10
        
        if (climb_m_s < 5) climb_m_s = 5;
        if (climb_m_s > 50) climb_m_s = 50;
        
        // Частота: 600 - (speed - 0.5) * 80
        sound.frequency = 600 - ((climb_m_s - 5) * 80) / 10;
        if (sound.frequency < 200) sound.frequency = 200;
        if (sound.frequency > 600) sound.frequency = 600;
        
        // Период прерывистости: 500 + (speed - 0.5) * 200
        sound.period_ms = 500 + ((climb_m_s - 5) * 200) / 10;
        if (sound.period_ms < 500) sound.period_ms = 500;
        if (sound.period_ms > 1500) sound.period_ms = 1500;
        
        sound.enabled = true;
        
        // Переключение состояния звука по таймеру
        uint32_t now = millis();
        if ((now - sound.last_toggle) >= sound.period_ms) {
            sound.sound_on = !sound.sound_on;
            sound.last_toggle = now;
        }
    }
}

// ============================================================================
// Калибровка
// ============================================================================
void performCalibration() {
    Serial.println("Calibration started...");
    digitalWrite(LED_PIN, HIGH);
    
    // Усреднение 20 измерений
    float sumPressure = 0;
    int validReadings = 0;
    
    for (int i = 0; i < 20; i++) {
        if (BARO.readPressure()) {
            sumPressure += BARO.readPressure(MILLIBAR);
            validReadings++;
            delay(10);
        }
    }
    
    if (validReadings > 0) {
        basePressure = (int32_t)(sumPressure / validReadings);
        
        // Сброс фильтра Калмана
        kalmanInit(0);
        prevAltitude = 0;
        filteredClimb = 0;
        
        // Мигание светодиодом для подтверждения
        for (int i = 0; i < 5; i++) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
        
        Serial.print("Calibration complete. Base pressure: ");
        Serial.println(basePressure);
    }
    
    digitalWrite(LED_PIN, LOW);
}

// ============================================================================
// Обработка данных датчика
// ============================================================================
void processSensorData() {
    uint32_t now = millis();
    uint32_t dt = now - lastSampleTime;
    lastSampleTime = now;
    
    if (dt < 10) return;  // Минимальный интервал
    
    // Чтение данных с датчика
    if (!BARO.readPressure()) {
        return;
    }
    
    sensorData.pressure = BARO.readPressure();
//    sensorData.temperature = BARO.temperature();
    
    // Расчет высоты в см
    int32_t pressureInt = (int32_t)(sensorData.pressure * 100);  // в Па * 100 для точности
    int32_t baseInt = (int32_t)(basePressure * 100);
    
    // Аппроксимация высоты
    int32_t altitude = pressureToAltitude(pressureInt, baseInt);
    
    // Фильтр Калмана
    int32_t filteredAltitude = kalmanUpdate(altitude);
    sensorData.altitude = filteredAltitude;
    
    // Расчет вертикальной скорости
    int32_t deltaAltitude = filteredAltitude - prevAltitude;
    float dtSec = dt / 1000.0f;  // в секундах
    
    if (dtSec > 0.1f) dtSec = 0.1f;  // Ограничение
    
    int16_t climb = (int16_t)(deltaAltitude / dtSec);  // см/с
    
    // Фильтр скорости (экспоненциальное сглаживание)
    filteredClimb = (filteredClimb * 7 + climb) / 8;
    sensorData.climb = filteredClimb;
    
    prevAltitude = filteredAltitude;
    
    // Обновление звуковой эмуляции
    updateSoundFromClimb(filteredClimb);
    
    // Отправка данных по BLE
    sendNMEAData((int)filteredAltitude, (int)filteredClimb);
    
    // Отладка в Serial
    if (millis() % 1000 < 50) {
        Serial.print("Alt: ");
        Serial.print(filteredAltitude);
        Serial.print(" cm, Climb: ");
        Serial.print(filteredClimb);
        Serial.print(" cm/s, Press: ");
        Serial.print(sensorData.pressure);
        Serial.print(" hPa, Temp: ");
//        Serial.print(sensorData.temperature);
//        Serial.print(" °C");
        
        // Вывод звуковых параметров
        if (sound.enabled) {
            Serial.print(", Sound: ");
            Serial.print(sound.frequency);
            Serial.print(" Hz");
            if (sound.period_ms > 0) {
                Serial.print(", Period: ");
                Serial.print(sound.period_ms);
                Serial.print(" ms");
            }
        } else {
            Serial.print(", Sound: OFF");
        }
        Serial.println();
    }
}

// ============================================================================
// Отправка данных для XCSoar/LK8000
// ============================================================================
void sendNMEAData(int alt, int vspeed){
//void sendDataToXCSoar() {
//    if (!bleSerial.isConnected()) return;
    
    static uint32_t lastSend = 0;
    uint32_t now = millis();
    
    // Отправка каждые 200 мс (5 Гц)
    if ((now - lastSend) < 200) return;
    lastSend = now;
    
    // Формат NMEA для вариометра и альтиметра
    char nmea[64];
    
    // 1. Высота: $PGRMZ,altitude,f,3
    int32_t altMeters = sensorData.altitude / 100;
    int32_t altCm = sensorData.altitude % 100;
    
    snprintf(nmea, sizeof(nmea), "$PGRMZ,%d.%02d,f,3\r\n", 
             (int)altMeters, (int)altCm);
#if BLE_EN
    bleSerial.print(nmea);
#endif

    // 2. Вертикальная скорость: $PGRMZ,climb,m,3
    int32_t climbMS = sensorData.climb / 100;
    int32_t climbCm = abs(sensorData.climb % 100);
    char sign = (sensorData.climb >= 0) ? '+' : '-';
    
    snprintf(nmea, sizeof(nmea), "$PGRMZ,%c%d.%02d,m,3\r\n", 
             sign, (int)climbMS, (int)climbCm);
#if BLE_EN
bleSerial.print(nmea);
#endif

    
    // 3. Альтернативный формат для LK8000: $LK8EX1
    // pressure: hPa * 10, alt: m, climb: m/s * 10
    int32_t pressHPa = (int32_t)(sensorData.pressure * 10);
    int32_t altM = sensorData.altitude / 100;
    int32_t climbMS10 = sensorData.climb / 10;
    
    snprintf(nmea, sizeof(nmea), "$LK8EX1,%.1f,%d,%.1f\r\n",
             sensorData.pressure, (int)altM, sensorData.climb / 100.0f);
#if BLE_EN
    bleSerial.print(nmea);
#endif

}

// ============================================================================
// Обработчик кнопки калибровки
// ============================================================================
void checkCalibrationButton() {
    static uint32_t lastDebounce = 0;
    static bool lastState = HIGH;
    
    bool currentState = digitalRead(BUTTON_PIN);
    
    if (currentState == LOW && lastState == HIGH) {
        uint32_t now = millis();
        if ((now - lastDebounce) > 50) {  // Дебаунс
            performCalibration();
            lastDebounce = now;
        }
    }
    lastState = currentState;
}

// ============================================================================
// Настройка BLE
// ============================================================================
#if BLE_EN
void setupBLE() {
    Serial.println("Initializing BLE...");
    
    Bluefruit.begin();
    Bluefruit.setName("Variometer_NRF52");
    Bluefruit.setConnectCallback([](uint16_t conn_handle) {
        Serial.println("BLE Connected");
        digitalWrite(LED_PIN, HIGH);
    });
    Bluefruit.setDisconnectCallback([](uint16_t conn_handle, uint8_t reason) {
        Serial.println("BLE Disconnected");
        digitalWrite(LED_PIN, LOW);
    });
    
    // Настройка BLE UART сервиса
    bleSerial.begin();
    
    // Запуск рекламы
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleSerial);
    Bluefruit.Advertising.start(0);  // 0 = бесконечная реклама
    
    Serial.println("BLE Advertising started");
}
#endif
// ============================================================================
// Настройка Arduino
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Variometer with LPS22HB starting...");
    
    // Настройка светодиода и кнопки
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);
    
    // Инициализация датчика LPS22HB
    if (!BARO.begin()) {
        Serial.println("Failed to initialize LPS22HB sensor!");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(500);
        }
    }
    Serial.println("LPS22HB sensor initialized");
    
    // Настройка датчика (максимальная точность)
// Найти актуальные
    // BARO.setContinuousMode();
    // BARO.setODR(10);  // 10 Гц
    
    // Инициализация фильтра Калмана
    kalmanInit(0);
    prevAltitude = 0;
    filteredClimb = 0;
    lastSampleTime = millis();
    
    // Инициализация звуковых параметров
    sound.enabled = false;
    sound.frequency = 0;
    sound.period_ms = 0;
    sound.sound_on = false;
    sound.last_toggle = 0;
    
    // Калибровка при старте
    performCalibration();
    
    // Настройка BLE
#if BLE_EN
    setupBLE();
#endif

}

// ============================================================================
// Основной цикл
// ============================================================================
void loop() {
    // Обработка BLE
#if BLE_EN
    Bluefruit.update();
#endif

    
    // Проверка кнопки калибровки
    checkCalibrationButton();
    
    // Опрос датчика с интервалом
    static uint32_t lastSample = 0;
    uint32_t now = millis();
    
    if ((now - lastSample) >= SAMPLE_INTERVAL_MS) {
        processSensorData();
        lastSample = now;
    }
    
    // Обновление светодиода (индикация звука)
    if (sound.enabled && sound.sound_on) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
    
    // Маленькая задержка для экономии энергии
    delay(5);
}

// ============================================================================
// Дополнительные функции для совместимости с XCSoar
// ============================================================================

// Обработка команд с телефона
void serialEvent() {
#if BLE_EN
    while (bleSerial.available()) {
        char c = bleSerial.read();
#else
    while (Serial.available()) {
        char c = Serial.read();
#endif
        if (c == 'C' || c == 'c') {
            performCalibration();
        }
    }
}

// ============================================================================
// Альтернативный метод для более точного расчета высоты
// (используется, если требуется большая точность)
// ============================================================================
float calculateAltitude(float pressure, float seaLevelPressure) {
    // Барометрическая формула (с плавающей точкой)
    // Используется только при наличии FPU
    // Для nRF52810 без FPU используйте целочисленную версию выше
    return 44330.0f * (1.0f - pow(pressure / seaLevelPressure, 0.1903f));
}