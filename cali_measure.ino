#include <Wire.h>                         // I2C communication
#include <Adafruit_Sensor.h>              // Unified sensor lib
#include <Adafruit_BNO055.h>              // BNO055 driver
#include <utility/imumaths.h>             // IMU math utilities

#define AS5600_ADDR 0x36                  // I2C address of AS5600
#define RAW_ANGLE_MSB 0x0C                // MSB register for raw angle

Adafruit_BNO055 bno = Adafruit_BNO055(55);  // BNO055 sensor object

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(9600);                     // Start Serial
  Wire.begin();                           // Initialize I2C (SDA 20, SCL 21 on Mega)
  delay(1000);                            // Delay for stability

  if (!bno.begin()) {
    Serial.println(" BNO055 not detected. Check wiring.");
    while (1);                            // Stop execution if not found
  }

  bno.setExtCrystalUse(true);             // Better accuracy
  Serial.println("🔧 Calibrating BNO055...");
  //calibrateBNO055();                      // Call BNO055 calibration
  Serial.println("🔧 Calibrating AS5600...");
  calibrateAS5600();                      // Simulated calibration (optional)
}

// ==============================
// Main Loop
// ==============================
void loop() {
  readIMUData();                          // Read BNO055
  readAS5600();                           // Read AS5600
  delay(500);                             // Delay between reads
}

// ==============================
// BNO055 Calibration Function
// ==============================
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

    delay(1000); // Wait 1s before rechecking
  }
}

// ==============================
// AS5600 "Calibration" Function
// ==============================
// Note: AS5600 is factory-calibrated; no actual calib needed.
// This just ensures it responds.
void calibrateAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("✅ AS5600 Detected and Ready.");
  } else {
    Serial.println("❌ AS5600 Not Detected! Check wiring.");
    while (1); // Stop execution
  }
}

// ==============================
// Read BNO055 Data
// ==============================
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

// ==============================
// Read AS5600 Angle in Degrees
// ==============================
void readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);     // Start I2C
  Wire.write(RAW_ANGLE_MSB);               // Point to MSB of angle
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);        // Request 2 bytes: MSB + LSB
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    int raw_angle = ((msb << 8) | lsb) & 0x0FFF;  // 12-bit value

    float angle_deg = (raw_angle * 360.0) / 4096.0;  // Convert to degrees

    Serial.print("🧭 AS5600 Angle: ");
    Serial.print(angle_deg);
    Serial.println("°");
  } else {
    Serial.println("❌ Failed to read AS5600");
  }
}
