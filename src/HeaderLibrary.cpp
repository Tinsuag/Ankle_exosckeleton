#include "HeaderLibrary.h"

MCP2515 mcp2515(53);
struct can_frame canMsg;

// === MOTOR ENABLE FUNCTION ===
void motorEnable() {
    canMsg.can_id  = 0x140 + motor_id;
    canMsg.can_dlc = 8;
    canMsg.data[0] = CMD_MOTOR_RUN;
    for (int i = 1; i < 8; i++) canMsg.data[i] = 0x00;

    mcp2515.sendMessage(&canMsg);
    Serial.println("Motor enabled.");
}

// === SET ACCELERATION ===
void setAcceleration(int32_t accel) {
    canMsg.can_id  = 0x140 + motor_id;
    canMsg.can_dlc = 8;
    canMsg.data[0] = CMD_WRITE_ACCEL_RAM;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;

    canMsg.data[4] = (uint8_t)(accel & 0xFF);
    canMsg.data[5] = (uint8_t)((accel >> 8) & 0xFF);
    canMsg.data[6] = (uint8_t)((accel >> 16) & 0xFF);
    canMsg.data[7] = (uint8_t)((accel >> 24) & 0xFF);

    mcp2515.sendMessage(&canMsg);
    Serial.print("Acceleration set to: ");
    Serial.print(accel);
    Serial.println(" dps²");
}

// === SEND POSITION COMMAND ===
void sendPositionCommand(uint32_t degree) {
    struct can_frame tx;
    tx.can_id  = 0x140 + motor_id;
    tx.can_dlc = 8;
    tx.data[0] = 0xA3;
    tx.data[1] = 0x00;
    tx.data[2] = 0x00;
    tx.data[3] = 0x00;
    tx.data[4] = (uint8_t)(degree & 0xFF);
    tx.data[5] = (uint8_t)((degree >> 8) & 0xFF);
    tx.data[6] = (uint8_t)((degree >> 16) & 0xFF);
    tx.data[7] = (uint8_t)((degree >> 24) & 0xFF);

    mcp2515.sendMessage(&tx);
    Serial.print("Sent position command (0.001 deg): ");
    Serial.println(degree);

    delay(10);

    struct can_frame response;
    if (mcp2515.readMessage(&response) == MCP2515::ERROR_OK) {
        if (response.can_id == (0x140 + motor_id) && response.data[0] == 0xA3) {
            int16_t torque = (response.data[2] | (response.data[3] << 8));
            int16_t speed  = (response.data[4] | (response.data[5] << 8));
            int16_t angle  = (response.data[6] | (response.data[7] << 8));

            float torque_Nm = (torque * 33.0) / 2048.0;
            float angle_deg = (angle * 360.0) / 65535.0;

            Serial.print("Feedback → Torque: "); Serial.print(torque_Nm);
            Serial.print(" Nm | Speed: "); Serial.print(speed);
            Serial.print(" dps | Angle: "); Serial.println(angle_deg);
        }
    }
}

// === SEND SPEED COMMAND ===
void sendSpeedCommand(int32_t speed) {
    struct can_frame tx;
    tx.can_id  = 0x140 + motor_id;
    tx.can_dlc = 8;
    tx.data[0] = 0xA2;
    tx.data[1] = 0x00;
    tx.data[2] = 0x00;
    tx.data[3] = 0x00;
    tx.data[4] = (uint8_t)(speed & 0xFF);
    tx.data[5] = (uint8_t)((speed >> 8) & 0xFF);
    tx.data[6] = (uint8_t)((speed >> 16) & 0xFF);
    tx.data[7] = (uint8_t)((speed >> 24) & 0xFF);

    mcp2515.sendMessage(&tx);
    Serial.print("Sent speed command (dps): ");
    Serial.println(speed);

    delay(10);

    struct can_frame response;
    if (mcp2515.readMessage(&response) == MCP2515::ERROR_OK) {
        if (response.can_id == (0x140 + motor_id) && response.data[0] == 0xA2) {
            int16_t torque = (response.data[2] | (response.data[3] << 8));
            int16_t speed  = (response.data[4] | (response.data[5] << 8));
            int16_t angle  = (response.data[6] | (response.data[7] << 8));

            Serial.print("Feedback → Torque: "); Serial.print(torque);
            Serial.print(" | Speed: "); Serial.print(speed);
            Serial.print(" | Angle: "); Serial.println(angle);
        }
    }
}

// === READ STATUS (Status2 feedback) ===
void readMotorStatus() {
    canMsg.can_id  = 0x140 + motor_id;
    canMsg.can_dlc = 8;
    canMsg.data[0] = CMD_STATUS_2;
    for (int i = 1; i < 8; i++) canMsg.data[i] = 0x00;

    mcp2515.sendMessage(&canMsg);
    delay(10);

    struct can_frame response;
    if (mcp2515.readMessage(&response) == MCP2515::ERROR_OK) {
        if (response.can_id == (0x140 + motor_id) && response.data[0] == CMD_STATUS_2) {
            int16_t torque = (response.data[2] | (response.data[3] << 8));
            int16_t speed  = (response.data[4] | (response.data[5] << 8));
            int16_t angle  = (response.data[6] | (response.data[7] << 8));

            Serial.print("Motor Status → Torque: "); Serial.print(torque);
            Serial.print(" | Speed: "); Serial.print(speed);
            Serial.print(" | Angle: "); Serial.println(angle);
        }
    }
}

// === GEAR RATIO DISPLAY ===
void GearReductionRate(int ratio) {
    Serial.print("Gear Reduction Ratio: ");
    Serial.println(ratio);
}
