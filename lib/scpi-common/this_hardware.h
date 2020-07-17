#ifndef __HARDWARE_DEF_H_
#define __HARDWARE_DEF_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SparkFunLSM9DS1.h"
#include "BraccioV2.h"

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -11.58 // Declination (degrees) in Melbourne, AU.
#define IMU_DEVICE_0_M_ADDR 0x1E
#define IMU_DEVICE_0_AG_ADDR 0x6B
#define IMU_DEVICE_1_M_ADDR 0x1C
#define IMU_DEVICE_1_AG_ADDR 0x6A
#define PRINT_CALCULATED
#define PRINT_RAW

/// Definitions for the arm ///
#define GRIPPER_CLOSED 73
#define GRIPPER_OPENED 20

/// defining IO pins for the teensy ///


void run_stuff(void); //general dummy function for testing stuff.

void init_hardware(void); //initalises all hardware IO, pins, features, etc. (except serial)

bool is_imu_connected(int device_id);
float get_gyro_x(int device_id);
float get_gyro_y(int device_id);
float get_gyro_z(int device_id);
float get_accel_x(int device_id);
float get_accel_y(int device_id);
float get_accel_z(int device_id);
float get_mag_x(int device_id);
float get_mag_y(int device_id);
float get_mag_z(int device_id);

#endif /* __HARDWARE_DEF_H_ */