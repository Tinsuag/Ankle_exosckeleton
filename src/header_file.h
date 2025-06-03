#ifndef HEADER_FILE_H
#define HEADER_FILE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define AS5600_ADDR 0x36
#define RAW_ANGLE_MSB 0x0C

extern Adafruit_BNO055 bno;

// Function declarations
void calibrateBNO055();
void calibrateAS5600();
void readIMUData();
void readAS5600();

#endif
