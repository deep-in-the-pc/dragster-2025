#include "PicoHM01B0.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <pico/time.h>

// -----------------------------------------------------------------------------
// PIO program for clock generation

#define clock_wrap_target 0
#define clock_wrap 1

static const uint16_t clock_program_instructions[] = {
    0xe001, // 0: set pins, 1
    0xe000, // 1: set pins, 0
};

static const struct pio_program clock_program = {
    .instructions = clock_program_instructions,
    .length = 2,
    .origin = -1,
};

static inline pio_sm_config clock_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + clock_wrap_target, offset + clock_wrap);
    return c;
}

static void clock_program_init(PIO pio, uint sm, uint offset, uint pin_base) {
    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, 1, true);
    pio_gpio_init(pio, pin_base);
    pio_sm_config c = clock_program_get_default_config(offset);
    sm_config_set_set_pins(&c, pin_base, 1);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, 2);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// -----------------------------------------------------------------------------
// PIO program for image capture

#define image_wrap_target 1
#define image_wrap 3

static uint16_t image_program_instructions[] = {
    0x2010, // 0: wait 0 gpio, 16
    0x208E, // 1: wait 1 gpio, 14
    0x4001, // 2: in pins, 1
    0x200E, // 3: wait 0 gpio, 14
};

static const struct pio_program image_program = {
    .instructions = image_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline pio_sm_config image_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + image_wrap_target, offset + image_wrap);
    return c;
}

static uint image_program_init(PIO *pio, uint *sm, uint *offset, uint pin_d0, int pin_pclk, int pin_vsync, int bus4bit) {
    int bus_bits = bus4bit ? 4 : 1;

    // Dynamically rewrite the PIO code for the provided pins:
    image_program_instructions[0] = 0x2000 | pin_vsync;  // wait 0 gpio, pin_vsync
    image_program_instructions[1] = 0x2080 | pin_pclk;     // wait 1 gpio, pin_pclk
    image_program_instructions[2] = 0x4000 | bus_bits;     // in pins, bus_bits
    image_program_instructions[3] = 0x2000 | pin_pclk;     // wait 0 gpio, pin_pclk

    // Allocate a new program instance
    if (!pio_claim_free_sm_and_add_program(&image_program, pio, sm, offset))
        return 0;
    for(int i = 0; i < bus_bits; i++)
        pio_gpio_init(*pio, pin_d0 + i);
    pio_sm_set_consecutive_pindirs(*pio, *sm, pin_d0, bus_bits, false);
    pio_gpio_init(*pio, pin_pclk);
    pio_sm_set_consecutive_pindirs(*pio, *sm, pin_pclk, 1, false);
    pio_gpio_init(*pio, pin_vsync);
    pio_sm_set_consecutive_pindirs(*pio, *sm, pin_vsync, 1, false);

    pio_sm_config c = image_program_get_default_config(*offset);
    sm_config_set_in_pins(&c, pin_d0);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(*pio, *sm, *offset, &c);

    return 1;
}

// -----------------------------------------------------------------------------
// Bit-banging I2C functions

static const int sensor_address = 0x24;

static void small_pause(void) {
    sleep_us(1);
}

static void slow_gpio_put(int pin, int value) {
    gpio_put(pin, value);
    small_pause();
}

static void i2c_bus_start(PicoHM01B0 *cam) {
    slow_gpio_put(cam->config.i2c_dat_gpio, 1);
    slow_gpio_put(cam->config.i2c_clk_gpio, 1);
    slow_gpio_put(cam->config.i2c_dat_gpio, 0);
    slow_gpio_put(cam->config.i2c_clk_gpio, 0);
}

static void i2c_bus_stop(PicoHM01B0 *cam) {
    slow_gpio_put(cam->config.i2c_dat_gpio, 0);
    slow_gpio_put(cam->config.i2c_clk_gpio, 1);
    slow_gpio_put(cam->config.i2c_dat_gpio, 1);
}

static void i2c_bus_send_ack(PicoHM01B0 *cam) {
    slow_gpio_put(cam->config.i2c_dat_gpio, 0);
    slow_gpio_put(cam->config.i2c_clk_gpio, 0);
    slow_gpio_put(cam->config.i2c_clk_gpio, 1);
    slow_gpio_put(cam->config.i2c_clk_gpio, 0);
    slow_gpio_put(cam->config.i2c_dat_gpio, 0);
}

static int i2c_bus_write_byte(PicoHM01B0 *cam, int data) {
    int i, tem;
    for (i = 0; i < 8; i++) {
        slow_gpio_put(cam->config.i2c_dat_gpio, ((data << i) & 0x80) != 0);
        slow_gpio_put(cam->config.i2c_clk_gpio, 1);
        slow_gpio_put(cam->config.i2c_clk_gpio, 0);
    }
    gpio_set_dir(cam->config.i2c_dat_gpio, GPIO_IN);
    small_pause();
    slow_gpio_put(cam->config.i2c_clk_gpio, 1);
    tem = !gpio_get(cam->config.i2c_dat_gpio);
    slow_gpio_put(cam->config.i2c_clk_gpio, 0);
    gpio_set_dir(cam->config.i2c_dat_gpio, GPIO_OUT);
    return tem;
}

static int i2c_write_reg(PicoHM01B0 *cam, int regID, int regDat) {
    i2c_bus_start(cam);
    if (i2c_bus_write_byte(cam, sensor_address << 1) == 0)
        return 0;
    sleep_us(10);
    if (i2c_bus_write_byte(cam, regID >> 8) == 0)
        return 0;
    sleep_us(10);
    if (i2c_bus_write_byte(cam, regID) == 0)
        return 0;
    sleep_us(10);
    if (i2c_bus_write_byte(cam, regDat) == 0)
        return 0;
    i2c_bus_stop(cam);
    return 1;
}

// -----------------------------------------------------------------------------
// Camera and exposure functions

// Calculate optimal line length and line count
static void calc_optimal_length(PicoHM01B0 *cam) {
    int min_line_length, min_line_count;
    int clocks_per_frame, max_line_count;
    int best_diff, best_lc, best_ll;

    if (cam->binning_2x2) {
        min_line_length = 215;
        min_line_count = cam->qvga_mode ? 172 : 172;
    } else {
        min_line_length = 376;
        min_line_count = cam->qvga_mode ? 260 : 344;
    }

    clocks_per_frame = (cam->config.mclk_freq / cam->clock_div) / cam->frame_rate;
    max_line_count = (clocks_per_frame + min_line_length - 1) / min_line_length;

    best_diff = 0x10000000;
    best_lc = min_line_count;
    best_ll = min_line_length;

    for (int lc = max_line_count; lc >= min_line_count; lc--) {
        int ll_round_down = clocks_per_frame / lc;
        for (int ll = ll_round_down; ll <= ll_round_down + 1; ll++) {
            if (ll < min_line_length)
                continue;
            int cpf = lc * ll;
            int diff = abs(cpf - clocks_per_frame);
            if (diff < best_diff) {
                best_diff = diff;
                best_lc = lc;
                best_ll = ll;
            }
        }
    }

    cam->line_length = best_ll;
    cam->line_count  = best_lc;
    cam->actual_frame_rate = (float)(cam->config.mclk_freq / cam->clock_div) / (best_ll * best_lc);
}

// Sensor register structure (local to this file)
typedef struct {
    uint16_t reg;
    uint8_t val;
} sensor_reg;

static const sensor_reg camera_reset_regs[] = {
    { 0x0103, 0x00 }, { 0x0100, 0x00 }, { 0x0101, 0x00 },
    { 0x0350, 0x7F }, { 0x1000, 0x43 }, { 0x1001, 0x40 },
    { 0x1002, 0x32 }, { 0x1003, 0x08 }, { 0x1006, 0x01 },
    { 0x1007, 0x08 }, { 0x1008, 0x00 }, { 0x1009, 0xA0 },
    { 0x100A, 0x60 }, { 0x100B, 0x90 }, { 0x100C, 0x40 },
    { 0x2000, 0x05 }, { 0x2100, 0x01 }, { 0x2101, 0x5F },
    { 0x2102, 0x0A }, { 0x2103, 0x03 }, { 0x2104, 0x05 },
    { 0x2107, 0x02 }, { 0x2108, 0x03 }, { 0x2109, 0x03 },
    { 0x210A, 0x00 }, { 0x210B, 0x80 }, { 0x210C, 0x40 },
    { 0x210D, 0x20 }, { 0x210E, 0x00 }, { 0x210F, 0x00 },
    { 0x2110, 0x85 }, { 0x2111, 0x00 }, { 0x2112, 0x70 },
    { 0x2150, 0x02 }, { 0x3011, 0x70 }, { 0x3022, 0x01 },
    { 0x3044, 0x0A }, { 0x3045, 0x00 }, { 0x3047, 0x0A },
    { 0x3050, 0xC0 }, { 0x3051, 0x42 }, { 0x3052, 0x50 },
    { 0x3053, 0x00 }, { 0x3054, 0x03 }, { 0x3055, 0xF7 },
    { 0x3056, 0xF8 }, { 0x3057, 0x29 }, { 0x3058, 0x1F },
    { 0x3059, 0x22 }, { 0x3060, 0x20 }, { 0x3062, 0xCC },
    { 0x3064, 0x00 }, { 0x3065, 0x04 }, { 0x3067, 0x00 },
    { 0x3068, 0x20 }, { 0x0104, 0x01 },
};

static const sensor_reg camera_start_streaming_regs[] = {
    { 0x0340, 0x00 }, { 0x0341, 0x00 },
    { 0x0342, 0x00 }, { 0x0343, 0x00 },
    { 0x0383, 0x00 }, { 0x0387, 0x00 },
    { 0x0390, 0x00 }, { 0x1012, 0x00 },
    { 0x3010, 0x00 }, { 0x2105, 0x00 },
    { 0x2106, 0x00 }, { 0x0104, 0x01 },
    { 0x0100, 0x01 },
};

static void arducam_regs_write(PicoHM01B0 *cam, const sensor_reg *camera_regs, int count) {
    for (int i = 0; i < count; i++) {
        int reg = camera_regs[i].reg;
        uint8_t value = camera_regs[i].val;

        switch (reg) {
            case 0x0101:
                value = cam->config.flip_horizontal + cam->config.flip_vertical * 2;
                break;
            case 0x1012:
                value = cam->binning_2x2 ? 0x03 : 0x01;
                break;
            case 0x2105:
                value = (cam->line_count - 2) >> 8;
                break;
            case 0x2106:
                value = (cam->line_count - 2) & 0xFF;
                break;
            case 0x3010:
                value = (cam->qvga_mode && !cam->binning_2x2) ? 1 : 0;
                break;
            case 0x0340:
                value = cam->line_count >> 8;
                break;
            case 0x0341:
                value = cam->line_count & 0xFF;
                break;
            case 0x0342:
                value = cam->line_length >> 8;
                break;
            case 0x0343:
                value = cam->line_length & 0xFF;
                break;
            case 0x0383:
            case 0x0387:
                value = cam->binning_2x2 ? 0x03 : 0x01;
                break;
            case 0x0390:
                value = cam->binning_2x2 ? 0x03 : 0x00;
                break;
            case 0x3059:
                value = cam->config.bus_4bit ? 0x42 : 0x22;
                break;
            case 0x3060:
                value = 0x20 + (cam->clock_div < 8) + (cam->clock_div < 4);
                break;
        }

        if (i2c_write_reg(cam, reg, value) == 0)
            i2c_bus_stop(cam);

        if (reg == 0x0103)
            sleep_ms(200);
    }
}

static void set_clock_vars(PicoHM01B0 *cam) {
    if (cam->config.mclk_gpio >= 0)
        cam->config.mclk_freq = clock_get_hz(clk_sys) / 4;

    if ((cam->config.mclk_freq > 24000000) || (!cam->config.bus_4bit))
        cam->clock_div = 8;
    else if (cam->config.mclk_freq > 12000000)
        cam->clock_div = 4;
    else
        cam->clock_div = 2;
}

// -----------------------------------------------------------------------------
// Public API implementations

int PicoHM01B0_begin(PicoHM01B0 *cam, const PicoHM01B0_config *config) {
    if (cam->state != STATE_RESET)
        return 0;

    memcpy(&cam->config, config, sizeof(PicoHM01B0_config));
    set_clock_vars(cam);

    if (cam->config.mclk_gpio >= 0) {
        if (!pio_claim_free_sm_and_add_program(&clock_program, &cam->clock_pio, &cam->clock_pio_sm, &cam->clock_pio_offset))
            return 0;
        clock_program_init(cam->clock_pio, cam->clock_pio_sm, cam->clock_pio_offset, cam->config.mclk_gpio);
    }

    if (!image_program_init(&cam->data_pio, &cam->data_pio_sm, &cam->data_pio_offset,
                              cam->config.d0_gpio, cam->config.pclk_gpio, cam->config.vsync_gpio, cam->config.bus_4bit)) {
        if (cam->config.mclk_gpio >= 0)
            pio_remove_program_and_unclaim_sm(&clock_program, cam->clock_pio, cam->clock_pio_sm, cam->clock_pio_offset);
        return 0;
    }

    cam->dma_channel = dma_claim_unused_channel(true);

    gpio_init(cam->config.vsync_gpio);
    gpio_set_dir(cam->config.vsync_gpio, GPIO_IN);

    gpio_init(cam->config.i2c_clk_gpio);
    gpio_set_dir(cam->config.i2c_clk_gpio, GPIO_OUT);
    gpio_put(cam->config.i2c_clk_gpio, 1);
    gpio_init(cam->config.i2c_dat_gpio);
    gpio_set_dir(cam->config.i2c_dat_gpio, GPIO_OUT);
    gpio_put(cam->config.i2c_dat_gpio, 1);

    sleep_ms(50);
    arducam_regs_write(cam, camera_reset_regs, sizeof(camera_reset_regs) / sizeof(sensor_reg));

    cam->exp_auto = true;
    cam->exp_lines = -1;
    cam->exp_analog_gain = -1;
    cam->exp_digital_gain = -1;

    cam->state = STATE_BEGIN;
    return 1;
}

void PicoHM01B0_set_fixed_exposure(PicoHM01B0 *cam, float exposure_ms, int d_gain, int a_gain) {
    bool something_changed = false;

    if (cam->state < STATE_STREAMING)
        return;

    if (cam->exp_auto) {
        i2c_write_reg(cam, 0x2100, 0x00);
        cam->exp_auto = false;
        something_changed = true;
    }

    int lines = cam->line_count * (exposure_ms * cam->actual_frame_rate / 1000.0f);
    if (lines > (int)(cam->line_count - 2))
        lines = cam->line_count - 2;
    if (lines < 2)
        lines = 2;
    if (lines != cam->exp_lines) {
        if ((lines >> 8) != (cam->exp_lines >> 8))
            i2c_write_reg(cam, 0x0202, lines >> 8);
        if ((lines & 0xFF) != (cam->exp_lines & 0xFF))
            i2c_write_reg(cam, 0x0203, lines & 0xFF);
        cam->exp_lines = lines;
        something_changed = true;
    }

    if (a_gain < 0)
        a_gain = 0;
    if (a_gain > 7)
        a_gain = 7;
    if (cam->exp_analog_gain != a_gain) {
        i2c_write_reg(cam, 0x0205, a_gain << 4);
        cam->exp_analog_gain = a_gain;
        something_changed = true;
    }

    if (d_gain < 1)
        d_gain = 1;
    if (d_gain > 255)
        d_gain = 255;
    if (cam->exp_digital_gain != d_gain) {
        if ((d_gain >> 6) != (cam->exp_digital_gain >> 6))
            i2c_write_reg(cam, 0x020E, d_gain >> 6);
        if (((d_gain << 2) & 0xFF) != ((cam->exp_digital_gain << 2) & 0xFF))
            i2c_write_reg(cam, 0x020F, d_gain << 2);
        cam->exp_digital_gain = d_gain;
        something_changed = true;
    }

    if (something_changed)
        i2c_write_reg(cam, 0x0104, 0x01);
}

void PicoHM01B0_set_auto_exposure(PicoHM01B0 *cam) {
    if (cam->exp_auto)
        return;
    i2c_write_reg(cam, 0x2100, 0x01);
    cam->exp_auto = true;
}

void PicoHM01B0_start_streaming_resolution(PicoHM01B0 *cam, uint16_t width, uint16_t height, bool binning_2x2, bool qvga_mode) {
    if (cam->state >= STATE_STREAMING)
        return;

    // Set sensor mode flags
    cam->binning_2x2 = binning_2x2;
    cam->qvga_mode = qvga_mode;
    
    // Set the resolution directly: width = line_length, height = line_count.
    cam->line_length = width;
    cam->line_count  = height;
    
    // Calculate the actual frame rate based on the clock frequency, divider, and new resolution.
    // This is computed as: mclk frequency / (clock_div * (line_length * line_count)).
    cam->actual_frame_rate = (float)(cam->config.mclk_freq / cam->clock_div) / (width * height);

    // Write the sensor registers to update the streaming parameters.
    arducam_regs_write(cam, camera_start_streaming_regs, sizeof(camera_start_streaming_regs) / sizeof(sensor_reg));

    // Wait for the sensor's vsync to ensure proper synchronization before declaring streaming.
    int timeout = 10000000UL;
    while (!gpio_get(cam->config.vsync_gpio) && timeout)
        timeout--;

    cam->state = STATE_STREAMING;
}

void PicoHM01B0_start_streaming(PicoHM01B0 *cam, float frame_rate, bool binning_2x2, bool qvga_mode) {
    if (cam->state >= STATE_STREAMING)
        return;

    cam->frame_rate = frame_rate;
    cam->binning_2x2 = binning_2x2;
    cam->qvga_mode = qvga_mode;

    calc_optimal_length(cam);
    arducam_regs_write(cam, camera_start_streaming_regs, sizeof(camera_start_streaming_regs) / sizeof(sensor_reg));

    int timeout = 10000000UL;
    while (gpio_get(cam->config.vsync_gpio) == false && timeout)
        timeout--;

    cam->state = STATE_STREAMING;
}

void PicoHM01B0_stop_streaming(PicoHM01B0 *cam) {
    if (cam->state != STATE_STREAMING)
        return;

    i2c_write_reg(cam, 0x0100, 0x00);
    cam->state = STATE_BEGIN;
}

void PicoHM01B0_start_capture(PicoHM01B0 *cam, uint8_t *dest) {
    if (cam->state != STATE_STREAMING)
        return;

    const uint32_t image_buf_size = (PicoHM01B0_get_rows(cam) * PicoHM01B0_get_cols(cam)) / 4;

    dma_channel_config c = dma_channel_get_default_config(cam->dma_channel);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(cam->data_pio, cam->data_pio_sm, false));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

    dma_channel_configure(cam->dma_channel, &c, dest, &(cam->data_pio->rxf[cam->data_pio_sm]), image_buf_size, false);

    pio_sm_restart(cam->data_pio, cam->data_pio_sm);
    pio_sm_clkdiv_restart(cam->data_pio, cam->data_pio_sm);
    pio_sm_exec(cam->data_pio, cam->data_pio_sm, pio_encode_jmp(cam->data_pio_offset));

    dma_channel_start(cam->dma_channel);
    pio_sm_set_enabled(cam->data_pio, cam->data_pio_sm, true);
    cam->state = STATE_CAPTURING;
}

bool PicoHM01B0_is_frame_ready(PicoHM01B0 *cam) {
    if (cam->state != STATE_CAPTURING)
        return true;

    bool ret = !dma_channel_is_busy(cam->dma_channel);
    if (ret)
        PicoHM01B0_wait_for_frame(cam);

    return ret;
}

void PicoHM01B0_wait_for_frame(PicoHM01B0 *cam) {
    if (cam->state != STATE_CAPTURING)
        return;

    dma_channel_wait_for_finish_blocking(cam->dma_channel);
    pio_sm_set_enabled(cam->data_pio, cam->data_pio_sm, false);
    cam->state = STATE_STREAMING;
}
