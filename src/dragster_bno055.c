//
// Created by david-g15ws on 26-04-2024.
//
#include "dragster_bno055.h"
#include "dragster.h"
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"

const uint32_t flash_target_offset = 0x80000; // 512 KB offset
const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + flash_target_offset);
const size_t data_size = sizeof(struct bno055_accel_offset_t) +  // Size of the data you want to write, in bytes
                         sizeof(struct bno055_mag_offset_t) +
                         sizeof(struct bno055_gyro_offset_t) +
                         sizeof(u8);

// IMU

typedef struct bno055_t bno055_t;

bno055_euler_float_t eulerAngles, firstEulerAngles;
bno055_linear_accel_float_t linearAccelData;
bno055_gravity_float_t gravityData;
bno055_accel_float_t accelData;
bno055_mag_float_t magData;

imu_heading_t imu_heading;

float x_pos;
float x_vel;
float prev_vel;
float prev_accel;

char packetBuffer[200];
char configBuffer[200];
absolute_time_t current_time, prev_time;

bno055_t bno;
u8 first_data = 0;

u8 i2c_lock = 0;

volatile u8 new_data;

// Calibration data structure
typedef struct {
    struct bno055_accel_offset_t accel_offset;
    struct bno055_mag_offset_t mag_offset;
    struct bno055_gyro_offset_t gyro_offset;
    int valid_data;
} calibration_data_t;


bool imu_callback(repeating_timer_t *rt)
{
    float heading_error, kp = -10;
    double delta_time;
    new_data = 1;

    if(i2c_lock)
    {
        snprintf(packetBuffer, sizeof(packetBuffer), "I2C locked");
        return true;
    }

    current_time = get_absolute_time();
    u8 mag_calib_u8, accel_calib_u8, gyro_calib_u8, sys_calib_u8;

    // Read the sensor data
    i2c_lock = 1;
    get_calib_stat(&mag_calib_u8, &accel_calib_u8, &gyro_calib_u8, &sys_calib_u8);
    bno055_convert_float_euler_hpr_deg(&eulerAngles);
    //bno055_convert_float_gravity_xyz_msq(&gravityData);
    bno055_convert_float_linear_accel_xyz_msq(&linearAccelData);
    i2c_lock = 0;
    imu_heading.heading = eulerAngles.h;
    /*snprintf(packetBuffer, sizeof(packetBuffer), "%llu, %3.2f, %3.2f, %lf, %u, %u, %u, %u, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f",
             current_time, x_pos, x_vel, delta_time, mag_calib_u8, accel_calib_u8, gyro_calib_u8, sys_calib_u8,
             eulerAngles.h, eulerAngles.r, eulerAngles.p,
             linearAccelData.x, linearAccelData.y, linearAccelData.z,
             direction, motor, fan);*/

    return true; // keep repeating

}


void i2c_bno055_init(void)
{
    // IMU
    gpio_init(IMU_RST_PIN);
    gpio_set_dir(IMU_RST_PIN, GPIO_OUT);
    gpio_put(IMU_RST_PIN, 0);

    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, GPIO_IN);

    i2c_init(i2c1, 400000);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    bi_decl(bi_2pins_with_func(IMU_SDA_PIN, IMU_SCL_PIN, GPIO_FUNC_I2C));


    gpio_put(IMU_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(IMU_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(IMU_RST_PIN, 1);
    sleep_ms(500);

    printf("IMU PIN INIT DONE!\n");

    int8_t res = bno055_pico_init(&bno, i2c1, BNO055_I2C_ADDR1);
    if (res) {
        res = bno055_pico_init(&bno, i2c1, BNO055_I2C_ADDR2);
    }
    if (res) {
        printf("BNO055 inilization failed!\n");
    }
    printf("IMU INIT DONE!\n");
    sleep_ms(100);

    res = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    if (res) {
        printf("BNO055 power mode set failed!\n");
    }
    printf("IMU POWER MODE SET!\n");
    sleep_ms(100);

    // Config axis remap
    res = config_axis_remap();
    if (res) {
        printf("BNO055 Axis config failed\n");
    }
    printf("IMU AXIS REMAP SET!\n");
    sleep_ms(100);

    if(!set_calib_settings())
    {
        res = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
        if (res) {
            printf("BNO055 operation mode set failed!\n");
        }
    }
    else
    {
        printf("IMU NDOF MODE SET!\n");
    }
}

BNO055_RETURN_FUNCTION_TYPE config_axis_remap(void)
{
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    u8 prev_opmode_u8 = BNO055_OPERATION_MODE_CONFIG;
    s8 stat_s8 = BNO055_ERROR;
    u8 remap_axis_u8 = 0b00010010;// X = Z, Y = -X, Z = -Y
    u8 remap_sign_u8 = 0b00000000;// Z,-Y,-X
    stat_s8 = bno055_get_operation_mode(&prev_opmode_u8);
    if (stat_s8 == BNO055_SUCCESS)
    {
        if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
        {
            stat_s8 += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
        }


        if (stat_s8 == BNO055_SUCCESS)
        {
            /* Write the value of axis remap */
            com_rslt += bno.BNO055_BUS_WRITE_FUNC(bno.dev_addr,
                                                  BNO055_REMAP_AXIS_VALUE_REG,
                                                  &remap_axis_u8,
                                                  BNO055_GEN_READ_WRITE_LENGTH);
            /* Write the value of sign remap */
            com_rslt += bno.BNO055_BUS_WRITE_FUNC(bno.dev_addr,
                                                  BNO055_AXIS_MAP_SIGN_ADDR,
                                                  &remap_sign_u8,
                                                  BNO055_GEN_READ_WRITE_LENGTH);
        }
        else
        {
            com_rslt = BNO055_ERROR;
        }
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }
    if (prev_opmode_u8 != BNO055_OPERATION_MODE_CONFIG)
    {
        /* set the operation mode
         * of previous operation mode*/
        com_rslt += bno055_set_operation_mode(prev_opmode_u8);
    }

    return com_rslt;
}

BNO055_RETURN_FUNCTION_TYPE get_calib_stat(u8 *mag_calib_u8, u8 *accel_calib_u8, u8 *gyro_calib_u8, u8 *sys_calib_u8)
{
    /* Variable used to return value of
     * communication routine*/
    BNO055_RETURN_FUNCTION_TYPE com_rslt = BNO055_ERROR;
    u8 data_u8r = BNO055_INIT_VALUE;
    s8 stat_s8 = BNO055_ERROR;

    /*condition check for page, mag calib
     * available in the page zero*/
    if (bno.page_id != BNO055_PAGE_ZERO)
    {
        /* Write the page zero*/
        stat_s8 = bno055_write_page_id(BNO055_PAGE_ZERO);
    }
    if ((stat_s8 == BNO055_SUCCESS) || (bno.page_id == BNO055_PAGE_ZERO))
    {
        /* Read the mag calib stat_s8 */
        com_rslt = bno.BNO055_BUS_READ_FUNC(bno.dev_addr,
                                                  BNO055_MAG_CALIB_STAT_REG,
                                                  &data_u8r,
                                                  BNO055_GEN_READ_WRITE_LENGTH);
        *mag_calib_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_MAG_CALIB_STAT);
        *accel_calib_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_ACCEL_CALIB_STAT);
        *gyro_calib_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_GYRO_CALIB_STAT);
        *sys_calib_u8 = BNO055_GET_BITSLICE(data_u8r, BNO055_SYS_CALIB_STAT);
    }
    else
    {
        com_rslt = BNO055_ERROR;
    }

    return com_rslt;
}

void get_calib_settings(void) {
    if (i2c_lock) {
        snprintf(configBuffer, sizeof(configBuffer), "I2C locked");
        return;
    }

    // Read offsets and radius and snprintf to packetBuffer
    i2c_lock = 1;
    printf("Reading calibration data\n");
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    sleep_ms(100);

    calibration_data_t cal_data;

    printf("Reading gyro offset\n");
    bno055_read_gyro_offset(&cal_data.gyro_offset);
    sleep_ms(100);

    printf("Reading mag offset\n");
    bno055_read_mag_offset(&cal_data.mag_offset);
    sleep_ms(100);

    printf("Reading accel offset\n");
    bno055_read_accel_offset(&cal_data.accel_offset);
    sleep_ms(100);

    cal_data.valid_data = 1;
    printf("Calibration data read\n");
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    i2c_lock = 0;

    // Write the calibration data to flash
    /*uint32_t ints = save_and_disable_interrupts();
    const uint32_t sector_size = FLASH_SECTOR_SIZE;
    const uint32_t sector_number = flash_target_offset / sector_size;
    const uint32_t sector_start = sector_number * sector_size;

    // Ensure the flash sector is erased before writing
    flash_range_erase(sector_start, sector_size);

    // Write data
    flash_range_program(flash_target_offset, (uint8_t *)&cal_data, sizeof(cal_data));
    restore_interrupts(ints);*/

    snprintf(configBuffer, sizeof(configBuffer), "Read: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
             cal_data.accel_offset.x, cal_data.accel_offset.y, cal_data.accel_offset.z, cal_data.accel_offset.r,
             cal_data.mag_offset.x, cal_data.mag_offset.y, cal_data.mag_offset.z, cal_data.mag_offset.r,
             cal_data.gyro_offset.x, cal_data.gyro_offset.y, cal_data.gyro_offset.z);
}

u8 set_calib_settings(void) {
    if (i2c_lock) {
        snprintf(configBuffer, sizeof(configBuffer), "I2C locked");
        return 0;
    }

    // Read the calibration data from flash
    calibration_data_t cal_data;
    //memcpy(&cal_data, (void *)(XIP_BASE + flash_target_offset), sizeof(cal_data));

    cal_data.accel_offset.x = -15;
    cal_data.accel_offset.y = -10;
    cal_data.accel_offset.z = -180;
    cal_data.accel_offset.r = 1000;
    cal_data.mag_offset.x = -1425;
    cal_data.mag_offset.y = 3564;
    cal_data.mag_offset.z = 217;
    cal_data.mag_offset.r = 497;
    cal_data.gyro_offset.x = -2;
    cal_data.gyro_offset.y = 2;
    cal_data.gyro_offset.z = -4;
    cal_data.valid_data = 1;
    //print cal_data
    printf("Valid %i Calibration data: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", cal_data.valid_data,
           cal_data.accel_offset.x, cal_data.accel_offset.y, cal_data.accel_offset.z, cal_data.accel_offset.r,
           cal_data.mag_offset.x, cal_data.mag_offset.y, cal_data.mag_offset.z, cal_data.mag_offset.r,
           cal_data.gyro_offset.x, cal_data.gyro_offset.y, cal_data.gyro_offset.z);

    // Set offsets and radius if valid data is available
    if (cal_data.valid_data == 1) {
        i2c_lock = 1;
        printf("Setting calibration data\n");
        bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
        sleep_ms(100);

        printf("Setting gyro offset\n");
        bno055_write_gyro_offset(&cal_data.gyro_offset);
        sleep_ms(100);

        printf("Setting mag offset\n");
        bno055_write_mag_offset(&cal_data.mag_offset);
        sleep_ms(100);

        printf("Setting accel offset\n");
        bno055_write_accel_offset(&cal_data.accel_offset);
        sleep_ms(100);

        // Optionally reset valid_data if you don't want to reuse the data
        // cal_data.valid_data = 0;
        // flash_range_erase(flash_target_offset, FLASH_SECTOR_SIZE);
        // flash_range_program(flash_target_offset, (uint8_t *)&cal_data, sizeof(cal_data));

        bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
        i2c_lock = 0;
        snprintf(configBuffer, sizeof(configBuffer), "Write: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
                 cal_data.accel_offset.x, cal_data.accel_offset.y, cal_data.accel_offset.z, cal_data.accel_offset.r,
                 cal_data.mag_offset.x, cal_data.mag_offset.y, cal_data.mag_offset.z, cal_data.mag_offset.r,
                 cal_data.gyro_offset.x, cal_data.gyro_offset.y, cal_data.gyro_offset.z);
        printf("Calibration data written\n");
        return 1;
    } else {
        snprintf(configBuffer, sizeof(configBuffer), "No valid data to write");
        printf("No valid data to write\n");
        return 0;
    }
}