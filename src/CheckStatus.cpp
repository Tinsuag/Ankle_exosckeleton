#include "header_file.h"

void readIMUData() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  Serial.print("📐 Orientation - Yaw: "); Serial.print(euler.x());
  Serial.print(" | Pitch: "); Serial.print(euler.y());
  Serial.print(" | Roll: "); Serial.println(euler.z());

  Serial.print("📊 Accel [m/s^2] - X: "); Serial.print(accel.x());
  Serial.print(" | Y: "); Serial.print(accel.y());
  Serial.print(" | Z: "); Serial.println(accel.z());
}

void readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_MSB);
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    int raw_angle = ((msb << 8) | lsb) & 0x0FFF;
    float angle_deg = (raw_angle * 360.0) / 4096.0;

    Serial.print("🧭 AS5600 Angle: ");
    Serial.print(angle_deg);
    Serial.println("°");
  } else {
    Serial.println("❌ Failed to read AS5600");
  }
}
