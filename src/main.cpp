#include "header_file.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);

  if (!bno.begin()) {
    Serial.println("❌ BNO055 not detected. Check wiring.");
    while (1);
  }

  bno.setExtCrystalUse(true);

  Serial.println("Commands:");
  Serial.println("  B1 -> Calibrate BNO055");
  Serial.println("  A1 -> Check AS5600 connection");
  Serial.println("  B2 -> Read BNO055 orientation + acceleration");
  Serial.println("  A2 -> Read AS5600 angle");
  Serial.println("  A3 -> Read both AS5600 + BNO055");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "B1") calibrateBNO055();
    else if (cmd == "A1") calibrateAS5600();
    else if (cmd == "B2") readIMUData();
    else if (cmd == "A2") readAS5600();
    else if (cmd == "A3") {
      readIMUData();
      readAS5600();
    }
    else Serial.println("❓ Unknown command.");
  }
}
