#ifndef PICOHM01B0_H
#define PICOHM01B0_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration structure (formerly PicoHM01B0_config class)
typedef struct {
    uint i2c_dat_gpio;
    uint i2c_clk_gpio;
    uint vsync_gpio;
    uint d0_gpio;
    uint pclk_gpio;
    int  mclk_gpio;
    uint mclk_freq;
    bool bus_4bit;
    bool flip_horizontal;
    bool flip_vertical;
} PicoHM01B0_config;

// State enum
typedef enum {
    STATE_RESET = 0,
    STATE_BEGIN = 1,
    STATE_STREAMING = 2,
    STATE_CAPTURING = 3,
} PicoHM01B0_state;

// Main device structure (formerly PicoHM01B0 class)
typedef struct {
    PicoHM01B0_state state;
    PicoHM01B0_config config;
    float frame_rate;
    float actual_frame_rate;
    int clock_div;
    bool qvga_mode;
    bool binning_2x2;
    bool exp_auto;
    int exp_lines, exp_analog_gain, exp_digital_gain;

    // PIO and DMA resources
    PIO  data_pio;
    uint data_pio_sm;
    uint data_pio_offset;
    uint dma_channel;

    PIO  clock_pio;
    uint clock_pio_sm;
    uint clock_pio_offset;

    // Calculated timing parameters
    uint line_length, line_count;
} PicoHM01B0;

// Public API functions
int  PicoHM01B0_begin(PicoHM01B0 *cam, const PicoHM01B0_config *config);
void PicoHM01B0_start_streaming_resolution(PicoHM01B0 *cam, uint16_t width, uint16_t height, bool binning_2x2, bool qvga_mode);
void PicoHM01B0_start_streaming(PicoHM01B0 *cam, float frame_rate, bool binning_2x2, bool qvga_mode);
void PicoHM01B0_start_capture(PicoHM01B0 *cam, uint8_t *dest);
bool PicoHM01B0_is_frame_ready(PicoHM01B0 *cam);
void PicoHM01B0_wait_for_frame(PicoHM01B0 *cam);
void PicoHM01B0_stop_streaming(PicoHM01B0 *cam);
void PicoHM01B0_set_fixed_exposure(PicoHM01B0 *cam, float exposure_ms, int d_gain, int a_gain);
void PicoHM01B0_set_auto_exposure(PicoHM01B0 *cam);

// Inline getters (using the same formulas as in the original class)
static inline int PicoHM01B0_get_cols(const PicoHM01B0 *cam) {
    return cam->binning_2x2 ? 164 : 324;
}
static inline int PicoHM01B0_get_rows(const PicoHM01B0 *cam) {
    int ret = cam->qvga_mode ? 244 : 324;
    return cam->binning_2x2 ? ret / 2 : ret;
}
static inline int PicoHM01B0_get_line_length(const PicoHM01B0 *cam) {
    return cam->line_length;
}
static inline int PicoHM01B0_get_line_count(const PicoHM01B0 *cam) {
    return cam->line_count;
}

static inline float PicoHM01B0_get_actual_frame_rate_fps(const PicoHM01B0 *cam) {
    return cam->actual_frame_rate;
}
static inline float PicoHM01B0_get_transfer_period_ms(const PicoHM01B0 *cam) {
    return 1000.0f * (cam->line_length * PicoHM01B0_get_rows(cam)) / (cam->config.mclk_freq / 8.0f);
}
static inline float PicoHM01B0_get_blanking_period_ms(const PicoHM01B0 *cam) {
    return 1000.0f * (cam->line_length * (cam->line_count - PicoHM01B0_get_rows(cam))) / (cam->config.mclk_freq / 8.0f);
}

#ifdef __cplusplus
}
#endif

#endif // PICOHM01B0_H
