#include "header_file.h"

void calibrateBNO055() {
  uint8_t sys, gyro, accel, mag;

  while (true) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("Calib - SYS: "); Serial.print(sys);
    Serial.print(" | GYRO: "); Serial.print(gyro);
    Serial.print(" | ACCEL: "); Serial.print(accel);
    Serial.print(" | MAG: "); Serial.println(mag);

    if (sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
      Serial.println("✅ BNO055 Fully Calibrated!");
      break;
    }
    delay(1000);
  }
}

void calibrateAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("✅ AS5600 Detected and Ready.");
  } else {
    Serial.println("❌ AS5600 Not Detected! Check wiring.");
    while (1);
  }
}
