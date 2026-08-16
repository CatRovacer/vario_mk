// Sending NMEA data via Serial and BLE
void sendNMEAData(int h, int v) {
  // Формируем строку: $LK8EX1,altitude,vspeed*checksum
  char buffer[50];
  // float h = altitude; // уже с учетом смещения
  // float v = verticalSpeed;
  uint8_t checksum = 0;
  sprintf(buffer, "$LK8EX1,%d,%d", (int)h, (int)v);
  // Calculate XOR per bytes of string after $
  for (int i = 1; i < strlen(buffer); i++) {
    checksum ^= buffer[i];
  }
  char out[60];
  sprintf(out, "%s*%02X\n", buffer, checksum);

  // Send via regular Serial
  Serial.print(out);
  //  Send via  BLE
#if 0
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
