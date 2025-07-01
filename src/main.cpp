#include "HeaderLibrary.h"


void setup() {
  Serial.begin(115200);   // Start serial monitor for feedback
  SPI.begin();            // Start SPI for CAN controller communication

  // Initialize CAN controller
  mcp2515.reset();                                 // Reset MCP2515
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);      // Set CAN speed: 1Mbps, 8MHz crystal
  mcp2515.setNormalMode();                         // Normal transmission mode

  delay(100);
  Serial.println("=== System Initialized ===");

  // === STEP 1: ENABLE MOTOR ===
  //motorEnable();                 
  delay(100);

  // === STEP 2: SET ACCELERATION VALUE ===
 // setAcceleration((int32_t)input_accel_dps2);
  //delay(100);

  // === STEP 3: SEND POSITION COMMAND ===
  // Convert angle to 0.01 deg unit (e.g., 90° → 9000)
  //sendPositionCommand((uint16_t)(input_angle_deg * 100), (uint16_t)input_speed_dps, direction);
}

void loop() {
  Serial.print("inloop");
}
