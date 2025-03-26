#ifndef DRAGSTER_H
#define DRAGSTER_H
/* 
 * HM01B0 Camera Configuration
 */

 /* I2C */
#define HM01B0_SDA_PIN 4
#define HM01B0_SCL_PIN 5
#define HM01B0_I2C_PORT i2c0

/* PIO */
#define HM01B0_d0_pin 10
#define HM01B0_d1_pin 11
#define HM01B0_d2_pin 12
#define HM01B0_d3_pin 13
#define HM01B0_pclk_pin 14
#define HM01B0_hsync_pin 15
#define HM01B0_vsync_pin 16
#define HM01B0_mclk_pin 17
#define HM01B0_data_bits 4

/* Camera */
#define HM01B0_WIDTH 160
#define HM01B0_HEIGHT 120
#define FRAME_BUFFER_SIZE  (HM01B0_WIDTH * HM01B0_HEIGHT)  // Adjust if your resolution is different

#define SYS_CLK_KHZ          250000

#endif // DRAGSTER_H