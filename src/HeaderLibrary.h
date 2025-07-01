#ifndef HEADERLIBRARY_H_
#define HEADERLIBRARY_H_

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

/*
 * === CAN COMMAND DEFINITIONS (RMD-X8 PRO DATASHEET) ===
 * These commands control or query the motor. Adjust/add for your model as needed.
 */

#define CMD_MOTOR_OFF              0x80  // Stop motor operation
#define CMD_MOTOR_RUN              0x88  // Start or resume motor operation
#define CMD_TORQUE_CLOSED_LOOP     0xA1  // Torque control
#define CMD_SPEED_CLOSED_LOOP      0xA2  // Speed control
#define CMD_POSITION_CLOSED_LOOP1  0xA3  // Single-turn absolute position control
#define CMD_POSITION_CLOSED_LOOP2  0xA4  // Multi-turn relative position control
#define CMD_POSITION_CLOSED_LOOP3  0xA5  // Multi-turn absolute position control
#define CMD_POSITION_CLOSED_LOOP4  0xA6  // Position control with speed & direction

#define CMD_READ_PID_RAM           0x30  // Read PID parameters from RAM
#define CMD_WRITE_PID_RAM          0x31  // Write PID parameters to RAM
#define CMD_READ_ACCEL_RAM         0x33  // Read acceleration from RAM
#define CMD_WRITE_ACCEL_RAM        0x34  // Write acceleration to RAM
#define CMD_READ_PID_ROM           0x32  // Read PID from ROM
#define CMD_WRITE_PID_ROM          0x91  // Write PID to ROM
#define CMD_READ_ACCEL_ROM         0x92  // Read acceleration from ROM
#define CMD_WRITE_ACCEL_ROM        0x93  // Write acceleration to ROM

#define CMD_READ_MOTOR_STATUS1     0x9A  // Status: Temp, voltage, error flags
#define CMD_READ_MOTOR_STATUS2     0x9C  // Status: Temp, torque, speed, encoder
#define CMD_READ_MOTOR_STATUS3     0x9D  // Status: Phase currents
#define CMD_READ_ENCODER_SINGLE    0x90  // Single-turn encoder value
#define CMD_READ_MULTI_TURN        0x94  // Multi-turn position

#define CMD_CLEAR_MOTOR_ERROR      0x81  // Clear error flags
#define CMD_RESET_MOTOR            0x82  // Soft reset
#define CMD_FACTORY_RESET          0x83  // Factory reset

/*
 * === CAN INTERFACE SETUP ===
 */
extern MCP2515 mcp2515;
extern struct can_frame canMsg;

/*
 * === MOTOR CONFIGURATION PARAMETERS ===
 */
const uint8_t  motor_id         = 0x01;     // Motor CAN node ID
const float    input_angle_deg  = 90.0;     // Desired position in degrees
const float    input_speed_dps  = 150.0;    // Speed in degrees/sec
const float    input_accel_dps2 = 3000.0;   // Acceleration in deg/sec²
const uint8_t  direction        = 0x00;     // 0x00 = CW, 0x01 = CCW
const int      gearRatio        = 6;        // Gear reduction ratio

/*
 * === FUNCTION PROTOTYPES ===
 */
void motorEnable();
void setAcceleration(int32_t accel);
void sendPositionCommand(uint32_t degree);
void sendSpeedCommand(int32_t speed);
void readMotorStatus();
void GearReductionRate(int ratio);

#endif
