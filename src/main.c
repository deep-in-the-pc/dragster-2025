/**
 * Dragster 25
 * Dragster Code for FNR25
 * David Carvalho - 2025
 */

#include "pico/stdlib.h"
#include "hm01b0_driver.h"
#include "PicoHM01B0.h"
#include "telemetry_stream.h"
#include "dragster.h"
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/vreg.h"

#include "pico/cyw43_arch.h"

#define FRAME_BUFFER_SIZE  (HM01B0_WIDTH * HM01B0_HEIGHT)   // Example for a 320x240 resolution frame
#define MHZ 1000000                     // 1,000,000 microseconds per second

PicoHM01B0 camera;

uint8_t frame[324][324] __attribute__((aligned(4)));

int main() {
 
    //Initialise IO as we are using printf for debug
    stdio_init_all();

    vreg_set_voltage(11);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLK_KHZ, true);
    stdio_init_all(); 
    
    /*
    // initialize CYW43 driver
    if (cyw43_arch_init()) {
        printf("cyw43_arch_init() failed.\n");
        return -1;
    }

    sleep_ms(500);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    
    // set up the initial configuration: pin numbers, orientation
    PicoHM01B0_config config;
    memset(&config, 0, sizeof(config));
    config.i2c_dat_gpio = HM01B0_SDA_PIN;
    config.i2c_clk_gpio = HM01B0_SCL_PIN;
    config.vsync_gpio   = HM01B0_vsync_pin;
    config.d0_gpio      = HM01B0_d0_pin;
    config.pclk_gpio    = HM01B0_pclk_pin;
    config.mclk_gpio    = -1;
    config.mclk_freq    = 24000000;
    config.bus_4bit     = true;
    config.flip_vertical   = true;
    config.flip_horizontal = true;



    // Initialize the camera
    if (!PicoHM01B0_begin(&camera, &config)) {
        // Handle initialization error here (e.g., print error message or blink an LED)
        return 1;
    }

    // Start streaming at 25 Hz, full resolution
    PicoHM01B0_start_streaming(&camera, 60, true, true);

    // Immediately start capture of the first frame
    PicoHM01B0_start_capture(&camera, &frame[0][0]);

    sleep_ms(500);   
    printf("Sensor initialized successfully!\n");
    
    
    // Allocate frame buffer
    memset(frame, 0, sizeof(frame));
    // Main loop: continuously capture a frame and print some debug info.
    while (true) {
        PicoHM01B0_wait_for_frame(&camera);
        //hm01b0_read_frame(frame_buffer, FRAME_BUFFER_SIZE);
        //print first 10 bytes of the frame buffer
        PicoHM01B0_start_capture(&camera, &frame[0][0]);

        for (int i = 0; i < 10000; i+=1000) {
            printf("%02x ", frame[i]);
        }
        printf("Frame captured.\n");
        //sleep_ms((1000/3));
    }*/
    
    // Launch the streaming on core1.
    telemetry_stream_start("Myhouse-Mesh", "Wolf!410", 1234);

    // Core0 can do other processing or simply loop.
    while (true) {
        tight_loop_contents();
    }
    return 0;
}