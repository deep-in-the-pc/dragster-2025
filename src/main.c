/**
 * Dragster 25
 * Dragster Code for FNR25
 * David Carvalho - 2025
 */

#include "pico/stdlib.h"
#include "PicoHM01B0.h"
#include "dragster_bno055.h"
#include "telemetry_stream.h"
#include "dragster.h"
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/vreg.h"

#include "pico/cyw43_arch.h"

#define MHZ 1000000                     // 1,000,000 microseconds per second
#define WIFI_DEBUG 0                    // Set to 1 to enable debug messages

PicoHM01B0 camera;

uint8_t frame[324][324] __attribute__((aligned(4)));

void camera_init(void)
{
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
    config.flip_vertical   = false;
    config.flip_horizontal = false;

    // Initialize the camera
    if (!PicoHM01B0_begin(&camera, &config)) {
        // Handle initialization error here (e.g., print error message or blink an LED)
        while (true) {
            tight_loop_contents();
        }
    }

    PicoHM01B0_set_auto_exposure(&camera);

    // Start streaming at 60 Hz, QVGA binning
    PicoHM01B0_start_streaming(&camera, 162, true, true);
    //PicoHM01B0_start_streaming_resolution(&camera, 0xd7, 0x80, false, false);

}

int main() {
 
    //Initialise IO as we are using printf for debug
    stdio_init_all();

    vreg_set_voltage(11);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLK_KHZ, true);
    stdio_init_all(); 

    camera_init();
    i2c_bno055_init();

    // Launch the streaming on core1.
    if(WIFI_DEBUG)
        telemetry_stream_start(WIFI_SSID, WIFI_PASSWORD, 1234);

    


    
    // Core0 can do other processing or simply loop.
    while (true) {
        tight_loop_contents();
    }
    return 0;
}