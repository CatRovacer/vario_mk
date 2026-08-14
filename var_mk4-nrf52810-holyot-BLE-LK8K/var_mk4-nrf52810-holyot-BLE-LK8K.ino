/*****************************************************************************
 * Вариометр и альтиметр на Holyiot nRF52810 + LPS22HB
 * Эмуляция звука Brauniger IQ ONE
 * Фильтр Калмана, калибровка, BLE UART (Nordic UART Service)
 * Для XCSoar / LK8000
 *****************************************************************************/
#define BLE_EN 0

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_LPS22HB.h>
//#include <Adafruit_TinyUSB.h>
#include <ArduinoBLE.h>

//#include <bluefruit.h>

// ============================================================================
// Конфигурация
// ============================================================================
#define BUZZER_PIN          17    // GPIO17 на Holyiot
#define CALIB_BUTTON_PIN    15    // GPIO15 для кнопки калибровки

#define SAMPLE_INTERVAL_MS  50    // 20 Гц выборка
#define ALTITUDE_SMOOTHING  0.7f  // Коэффициент сглаживания (без float - используем целые)

// Параметры фильтра Калмана (масштабированные)
#define KALMAN_Q_SCALED     10    // Q * 1000
#define KALMAN_R_SCALED     500   // R * 1000
#define KALMAN_INIT_P       1000  // Начальная ковариация * 1000

// ============================================================================
// Глобальные переменные
// ============================================================================
//LPS22HB BARO;

// Данные датчика
static int32_t g_pressure = 0;        // давление в Па * 10
static int32_t g_temperature = 0;     // температура в °C * 100
static int32_t g_altitude = 0;        // высота в см
static int32_t g_altitude_filtered = 0;
static int16_t g_climb_filtered = 0;  // вертикальная скорость в см/с

// Калибровка
static int32_t g_base_pressure = 0;   // давление на земле * 10
static int32_t g_base_altitude = 0;   // высота на земле в см

// Фильтр Калмана (целочисленная версия)
static int32_t kalman_x = 0;          // состояние
static int32_t kalman_p = KALMAN_INIT_P; // ковариация * 1000
static int32_t kalman_k = 0;          // коэффициент Калмана * 1000

// Тайминги
static uint32_t g_last_sample_time = 0;
static int32_t g_prev_altitude = 0;
static uint32_t g_sample_counter = 0;

// Звук
static bool g_sound_enabled = false;
static uint16_t g_sound_freq = 0;
static uint16_t g_sound_period_ms = 0;
static uint32_t g_sound_last_toggle = 0;
static bool g_sound_state = false;

// BLE
static bool g_ble_connected = false;
static uint32_t g_last_ble_send = 0;

// Флаг калибровки
static volatile bool g_calibration_requested = false;

// ============================================================================
// Фильтр Калмана (целочисленный, без float)
// ============================================================================
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

// ============================================================================
// Расчет высоты по давлению (без float)
// ============================================================================
int32_t calculate_altitude(int32_t pressure, int32_t base_pressure) {
    // Барометрическая формула: h = 44330 * (1 - (p/p0)^0.1903)
    // Используем аппроксимацию для целочисленных вычислений
    
    if (pressure <= 0 || base_pressure <= 0) return 0;
    
    // Преобразуем к общему масштабу (давление * 10)
    int32_t p = pressure / 10;
    int32_t p0 = base_pressure / 10;
    
    // Вычисляем отношение p/p0 * 10000 для целочисленной арифметики
    int32_t ratio = (p * 10000) / p0;
    
    // Таблица приближений для (1 - ratio^0.1903)
    // Используем линейную интерполяцию для ratio от 500 до 1000
    // (соответствует высоте от 0 до 5500 м)
    
    static const int16_t alt_table[51][2] = {
        {1000, 0},   {990, 84},   {980, 168},  {970, 253},  {960, 338},
        {950, 424},  {940, 510},  {930, 597},  {920, 684},  {910, 771},
        {900, 859},  {890, 948},  {880, 1037}, {870, 1127}, {860, 1218},
        {850, 1309}, {840, 1401}, {830, 1494}, {820, 1588}, {810, 1682},
        {800, 1777}, {790, 1873}, {780, 1970}, {770, 2068}, {760, 2166},
        {750, 2266}, {740, 2366}, {730, 2468}, {720, 2570}, {710, 2674},
        {700, 2778}, {690, 2884}, {680, 2991}, {670, 3099}, {660, 3208},
        {650, 3318}, {640, 3430}, {630, 3543}, {620, 3657}, {610, 3773},
        {600, 3890}, {590, 4009}, {580, 4129}, {570, 4250}, {560, 4373},
        {550, 4498}, {540, 4624}, {530, 4752}, {520, 4882}, {510, 5013},
        {500, 5146}
    };
    
    // Находим интервал в таблице
    int32_t alt = 0;
    for (int i = 0; i < 50; i++) {
        if (ratio >= alt_table[i+1][0]) {
            // Линейная интерполяция
            int32_t r1 = alt_table[i][0];
            int32_t r2 = alt_table[i+1][0];
            int32_t a1 = alt_table[i][1];
            int32_t a2 = alt_table[i+1][1];
            
            alt = a1 + ((a2 - a1) * (ratio - r1)) / (r2 - r1);
            break;
        }
    }
    
    // Переводим в сантиметры
    return alt * 100; // высота в см
}

// ============================================================================
// Обновление звука по алгоритму Brauniger IQ ONE
// ============================================================================
void update_sound(int16_t climb_cm_s) {
    // Climb в см/с (положительный - подъем)
    
    if (abs(climb_cm_s) < 10) {
        // Нейтральная зона - тишина
        g_sound_enabled = false;
        tone(BUZZER_PIN, 0);
        return;
    }
    
    uint16_t freq = 0;
    uint16_t period_ms = 0;
    int16_t abs_climb = abs(climb_cm_s);
    
    if (climb_cm_s > 0) {
        // Подъем: непрерывный тон, частота растет
        // От 600 Гц при 0.5 м/с до 1200 Гц при 5 м/с
        int16_t climb_m_s = abs_climb / 100;
        if (climb_m_s < 5) climb_m_s = 5;
        if (climb_m_s > 50) climb_m_s = 50;
        
        freq = 600 + ((climb_m_s - 5) * 133) / 10;
        if (freq < 600) freq = 600;
        if (freq > 1200) freq = 1200;
        
        g_sound_enabled = true;
        g_sound_state = true;
        tone(BUZZER_PIN, freq);
        
    } else {
        // Снижение: прерывистый тон
        int16_t climb_m_s = abs_climb / 100;
        if (climb_m_s < 5) climb_m_s = 5;
        if (climb_m_s > 50) climb_m_s = 50;
        
        freq = 600 - ((climb_m_s - 5) * 80) / 10;
        if (freq < 200) freq = 200;
        
        // Период прерываний пропорционален скорости
        period_ms = 500 + ((climb_m_s - 5) * 200) / 10;
        if (period_ms > 1500) period_ms = 1500;
        
        // Таймер прерываний
        uint32_t now = millis();
        if ((now - g_sound_last_toggle) > period_ms) {
            g_sound_state = !g_sound_state;
            g_sound_last_toggle = now;
        }
        
        if (g_sound_state) {
            g_sound_enabled = true;
            tone(BUZZER_PIN, freq);
        } else {
            g_sound_enabled = false;
            tone(BUZZER_PIN, 0);
        }
    }
}

// ============================================================================
// Обработка данных датчика
// ============================================================================
void process_sensor_data() {
    uint32_t now = millis();
    uint32_t dt = now - g_last_sample_time;
    
    if (dt < SAMPLE_INTERVAL_MS) return;
    g_last_sample_time = now;
    
    // Читаем датчик
    float pressure_float = BARO.readPressure();
    float temp_float = BARO.readTemperature();
    
    // Конвертируем в целые (давление * 10, температура * 100)
    g_pressure = (int32_t)(pressure_float * 10.0f);
    g_temperature = (int32_t)(temp_float * 100.0f);
    
    if (g_pressure <= 0) return;
    
    // Расчет высоты
    int32_t altitude;
    if (g_base_pressure > 0) {
        altitude = calculate_altitude(g_pressure, g_base_pressure);
    } else {
        altitude = 0;
    }
    
    // Фильтр Калмана
    int32_t filtered_alt = kalman_update(altitude);
    g_altitude_filtered = filtered_alt;
    
    // Расчет вертикальной скорости (дифференцирование)
    float dt_sec = dt / 1000.0f;
    if (dt_sec > 0.5f) dt_sec = 0.5f;
    
    int32_t delta_alt = filtered_alt - g_prev_altitude;
    int16_t climb = (int16_t)(delta_alt / dt_sec); // см/с
    
    // Сглаживание скорости (фильтр низких частот)
    if (g_sample_counter < 2) {
        g_climb_filtered = climb;
    } else {
        g_climb_filtered = (g_climb_filtered * 7 + climb) / 8;
    }
    
    g_prev_altitude = filtered_alt;
    g_altitude = filtered_alt;
    g_sample_counter++;
    
    // Обновление звука
    update_sound(g_climb_filtered);
}

// ============================================================================
// BLE UART (Nordic UART Service)
// ============================================================================
#if BLE_EN
  BLEDis bledis;
  BLEBas blebas;
  BLEUart bleuart;

void ble_connect_callback(uint16_t conn_handle) {
    g_ble_connected = true;
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Звуковой сигнал подключения
    tone(BUZZER_PIN, 1000);
    delay(100);
    tone(BUZZER_PIN, 1500);
    delay(100);
    tone(BUZZER_PIN, 2000);
    delay(100);
    noTone(BUZZER_PIN);
}

void ble_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    g_ble_connected = false;
    digitalWrite(LED_BUILTIN, LOW);
}

void ble_uart_rx_callback(uint16_t conn_handle) {
    // Читаем команды с телефона
    while (bleuart.available()) {
        char c = (char)bleuart.read();
        if (c == 'C' || c == 'c') {
            g_calibration_requested = true;
        }
    }
}

void setup_ble() {
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("Holyiot Vario");
    
    // Настройка сервисов
    bledis.setManufacturer("Holyiot");
    bledis.setModel("nRF52810 Vario");
    Bluefruit.setDeviceName("Holyiot Vario");
    
    // Сервис батареи
    blebas.begin();
    
    // UART сервис
    bleuart.begin();
    bleuart.setRxCallback(ble_uart_rx_callback);
    
    // Callbacks подключения
    Bluefruit.setConnectCallback(ble_connect_callback);
    Bluefruit.setDisconnectCallback(ble_disconnect_callback);
    
    // Настройка рекламы
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleuart);
    Bluefruit.Advertising.start();
}
#endif

// ============================================================================
// Отправка данных в XCSoar/LK8000
// ============================================================================
void send_vario_data() {
#if BLE_EN
    if (!g_ble_connected) return;
#endif

    uint8_t i, checksum = 0;
    uint32_t now = millis();
    if ((now - g_last_ble_send) < 500) return; // 2 Гц
    g_last_ble_send = now;
    
    char buffer[64];
    char out[60];    
    // Формат NMEA для XCSoar:
    // $PGRMZ,altitude,f,3*XX - высота в футах
    // $PGRMZ,climb,m,3*XX - вертикальная скорость
    
    // Высота в футах (1 м = 3.28084 фута)
    int32_t alt_ft = (g_altitude * 328) / 10000; // см в футы
    
    // Вертикальная скорость в м/с
    int16_t climb_m_s = g_climb_filtered / 100;
    int16_t climb_cm = abs(g_climb_filtered % 100);
    
    // Высота
    sprintf(buffer, "$PGRMZ,%ld.%02d,f,3", g_altitude/100, g_altitude%100 /*alt_ft / 100, alt_ft % 100*/);
    for (i = 1; i < strlen(buffer); i++) {
    checksum ^= buffer[i];
    }
    sprintf(out, "%s*%02X\n", buffer, checksum);

#if BLE_EN
    bleuart.print(buffer);
#endif    
    // Вертикальная скорость
    if (g_climb_filtered >= 0) {
        sprintf(buffer, "$PGRMZ,%d.%02d,m,3", climb_m_s, climb_cm);
    } else {
        sprintf(buffer, "$PGRMZ,-%d.%02d,m,3", abs(climb_m_s), climb_cm);
    }
    for (i = 1; i < strlen(buffer); i++) {
    checksum ^= buffer[i];
    }
    sprintf(out, "%s*%02X\n", buffer, checksum);
    Serial.print(out);

#if BLE_EN
    bleuart.print(buf);
#endif    
    
    // Также отправляем в формате LXWP для LK8000
    // $LXWP0,logger,time,altitude,climb,speed,...
    sprintf(buffer, "$LXWP0,N,%02d:%02d:%02d,%ld.%02d,%d.%02d,0.0,0,0,0",
            (now / 3600000) % 24,
            (now / 60000) % 60,
            (now / 1000) % 60,
            g_altitude / 100, g_altitude % 100,
            g_climb_filtered / 100, abs(g_climb_filtered % 100));
    for (i = 1; i < strlen(buffer); i++) {
    checksum ^= buffer[i];
    }
    sprintf(out, "%s*%02X\n", buffer, checksum);
    Serial.print(out);

#if BLE_EN
    bleuart.print(buffer);
#endif    
}

// ============================================================================
// Калибровка
// ============================================================================
void perform_calibration() {
    // Берем 20 измерений и усредняем
    int32_t sum_pressure = 0;
    int32_t sum_altitude = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < 20; i++) {
        float p = BARO.readPressure();
        if (p > 0) {
            sum_pressure += (int32_t)(p * 10.0f);
            valid_samples++;
        }
        delay(20);
    }
    
    if (valid_samples > 0) {
        g_base_pressure = sum_pressure / valid_samples;
        
        // Сбрасываем фильтр Калмана
        kalman_init(0);
        g_prev_altitude = 0;
        g_climb_filtered = 0;
        g_altitude_filtered = 0;
        g_sample_counter = 0;
        
        // Звуковой сигнал калибровки
        for (int i = 0; i < 3; i++) {
            tone(BUZZER_PIN, 1000 + i * 200);
            delay(150);
        }
        noTone(BUZZER_PIN);
        
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
    }
}

// ============================================================================
// Обработчик кнопки
// ============================================================================
void button_handler() {
    static uint32_t last_press = 0;
    uint32_t now = millis();
    
    // Защита от дребезга
    if ((now - last_press) < 200) return;
    last_press = now;
    
    g_calibration_requested = true;
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    // Инициализация пинов
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    
    digitalWrite(LED_BUILTIN, LOW);
    
    // Инициализация датчика давления
    if (!BARO.begin()) {
        // Ошибка датчика - мигаем светодиодом
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(900);
        }
    }
    
    // Настройка датчика
    // BARO.setDataRate(LPS22HB_RATE_25HZ);
    // BARO.setMode(LPS22HB_MODE_CONT);
    // BARO.setLowPassFilter(false);
    
    delay(100);
    
    // Инициализация BLE
#if BLE_EN
    setup_ble();
#endif    
    
    // Начальная калибровка
    perform_calibration();
    
    // Таймеры
    g_last_sample_time = millis();
    g_last_ble_send = millis();
    g_sound_last_toggle = millis();
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
    // Обработка кнопки
    if (digitalRead(CALIB_BUTTON_PIN) == LOW) {
        button_handler();
    }
    
    // Обработка калибровки
    if (g_calibration_requested) {
        g_calibration_requested = false;
        perform_calibration();
    }
    
    // Обработка датчика
    process_sensor_data();
    
    // Отправка данных по BLE
    send_vario_data();
    
    // Маленькая задержка для стабильности
    delay(5);
    
    // Выход в сон для экономии энергии (nRF52810)
    // Bluefruit.sleep();
}