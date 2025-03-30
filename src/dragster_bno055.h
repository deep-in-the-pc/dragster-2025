//
// Created by david-g15ws on 26-04-2024.
//

#ifndef DRAGSTER_DRAGSTER_BNO055_H
#define DRAGSTER_DRAGSTER_BNO055_H

#define RASPBERRYPI_PICO
#include "PicoBNO055.h"
typedef struct bno055_accel_float_t bno055_accel_float_t;
typedef struct bno055_euler_float_t bno055_euler_float_t;
typedef struct bno055_mag_float_t bno055_mag_float_t;
typedef struct bno055_linear_accel_float_t bno055_linear_accel_float_t;
typedef struct bno055_gravity_float_t bno055_gravity_float_t;

typedef struct imu_heading_t {
    float heading;
    float target_heading;
    float error;
} imu_heading_t;

extern imu_heading_t imu_heading;

extern bno055_euler_float_t eulerAngles;
extern bno055_linear_accel_float_t linearAccelData;
extern bno055_accel_float_t accelData;
extern bno055_mag_float_t magData;
extern char packetBuffer[200];
extern char configBuffer[200];

extern float x_pos;
extern float x_vel;


extern absolute_time_t current_time;

extern u8 i2c_lock;

void i2c_bno055_init(void);

void setup_interrupts(void);

bool imu_callback(repeating_timer_t *rt);

BNO055_RETURN_FUNCTION_TYPE config_axis_remap(void);
BNO055_RETURN_FUNCTION_TYPE get_calib_stat(u8 *mag_calib_u8, u8 *accel_calib_u8, u8 *gyro_calib_u8, u8 *sys_calib_u8);
void get_calib_settings(void);
u8 set_calib_settings(void);

extern volatile u8 new_data;

#endif //DRAGSTER_DRAGSTER_BNO055_H