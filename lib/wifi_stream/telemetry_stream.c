#include "telemetry_stream.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "PicoHM01B0.h"      // Updated header for new HM01B0 driver
#include "dragster.h"         // Other dependency as needed
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHUNK_SIZE         1460         // Send in chunks.
#define TCP_PORT           4242         // Listening port.
#define POLL_TIME_S        5
#define FRAME_BUFFER_SIZE  (324 * 324)  // Define as needed

// Global camera instance – assumed to be configured and started elsewhere.
static PicoHM01B0 camera;

typedef struct {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    uint8_t *frame_buffer;
    uint32_t frame_size;
    uint32_t sent_offset;
} telemetry_state_t;

static telemetry_state_t *telemetry_state = NULL;

static void wait_for_send_buffer(struct tcp_pcb *tpcb, int required) {
    while (tcp_sndbuf(tpcb) < required) {
        // Delay a short period to allow the stack to free up space.
        sleep_ms(1);
    }
}

static err_t send_frame_packet(struct tcp_pcb *tpcb, telemetry_state_t *state) {
    // Get current resolution from the camera.
    uint32_t width, height;
    uint8_t padding_bytes = 4; 
    if (camera.qvga_mode) {
        if (camera.binning_2x2) {
            width = 160;
            height = 120;
        } else {
            width = 320;
            height = 240;
        }
    } else {
        if (camera.binning_2x2) {
            width = 160;
            height = 160;
        } else {
            width = 320;
            height = 320;
        }
    }
    state->frame_size = (width+padding_bytes) * (height+padding_bytes);  // Total image bytes.

    // Prepare an 8-byte resolution header (width and height as 32-bit integers in network order)
    uint32_t res_header[2];
    res_header[0] = htonl(width+padding_bytes);
    res_header[1] = htonl(height+padding_bytes);

    // Define the total packet length: header (8 bytes) + image data.
    uint32_t packet_len = 8 + state->frame_size;
    uint32_t net_packet_len = htonl(packet_len);

    // Send the packet length header (4 bytes).
    err_t ret = tcp_write(tpcb, &net_packet_len, sizeof(net_packet_len), TCP_WRITE_FLAG_COPY);
    if (ret != ERR_OK) {
        printf("tcp_write packet length failed: %d\n", ret);
        return ret;
    }
    tcp_output(tpcb);

    // Send the resolution header (8 bytes).
    ret = tcp_write(tpcb, res_header, sizeof(res_header), TCP_WRITE_FLAG_COPY);
    if (ret != ERR_OK) {
        printf("tcp_write resolution header failed: %d\n", ret);
        return ret;
    }
    tcp_output(tpcb);

    // Now send the image data in chunks.
    state->sent_offset = 0;
    int to_send = (state->frame_size > CHUNK_SIZE) ? CHUNK_SIZE : state->frame_size;
    ret = tcp_write(tpcb, state->frame_buffer, to_send, TCP_WRITE_FLAG_COPY);
    if (ret != ERR_OK) {
        printf("tcp_write image data failed: %d\n", ret);
        return ret;
    }
    tcp_output(tpcb);
    return ERR_OK;
}

/* Callback: Called when data previously written has been acknowledged */
static err_t telemetry_tcp_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    telemetry_state_t *state = (telemetry_state_t *)arg;
    state->sent_offset += len;
    if (state->sent_offset < state->frame_size) {
        int remaining = state->frame_size - state->sent_offset;
        int to_send = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        
        // Wait until there's enough space in the TCP send buffer.
        wait_for_send_buffer(tpcb, to_send);
        
        err_t err = tcp_write(tpcb, state->frame_buffer + state->sent_offset, to_send, TCP_WRITE_FLAG_COPY);
        if (err != ERR_OK) {
            printf("tcp_write image data failed: %d\n", err);
            return err;
        }
        tcp_output(tpcb);
    } else {
        // Entire frame has been sent; capture the next frame.
        state->sent_offset = 0;
        PicoHM01B0_start_capture(&camera, state->frame_buffer);
        PicoHM01B0_wait_for_frame(&camera);
        
        // Before sending the new frame packet, wait for buffer space if needed.
        // (send_frame_packet() will do its own checks if needed)
        err_t err = send_frame_packet(tpcb, state);
        if (err != ERR_OK) {
            return err;
        }
    }
    return ERR_OK;
}
/* Callback: Called when data is received from the client (if any) */
static err_t telemetry_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p != NULL) {
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
    }
    return ERR_OK;
}

/* Callback: Polling (optional) */
static err_t telemetry_tcp_poll(void *arg, struct tcp_pcb *tpcb) {
    // Here you could check connection health.
    return ERR_OK;
}

/* Callback: Called when an error occurs */
static void telemetry_tcp_err(void *arg, err_t err) {
    telemetry_state_t *state = (telemetry_state_t *)arg;
    printf("Connection error: %d\n", err);
    if (state && state->client_pcb) {
        tcp_abort(state->client_pcb);
        state->client_pcb = NULL;
    }
}

/* Callback: Accept a new client connection */
static err_t telemetry_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    telemetry_state_t *state = (telemetry_state_t *)arg;
    if (!newpcb) {
        return ERR_VAL;
    }
    state->client_pcb = newpcb;
    tcp_arg(newpcb, state);
    tcp_sent(newpcb, telemetry_tcp_sent);
    tcp_recv(newpcb, telemetry_tcp_recv);
    tcp_poll(newpcb, telemetry_tcp_poll, POLL_TIME_S);
    tcp_err(newpcb, telemetry_tcp_err);

    // Allocate or reallocate frame buffer based on current camera resolution.
    uint32_t width, height;
    uint8_t padding_bytes = 4; 
    if (camera.qvga_mode) {
        if (camera.binning_2x2) {
            width = 160;
            height = 120;
        } else {
            width = 320;
            height = 240;
        }
    } else {
        if (camera.binning_2x2) {
            width = 160;
            height = 160;
        } else {
            width = 320;
            height = 320;
        }
    }
    state->frame_size = (width+padding_bytes) * (height+padding_bytes);  // Total image bytes.

    if (state->frame_buffer) {
        free(state->frame_buffer);
    }
    state->frame_buffer = malloc(state->frame_size);
    if (!state->frame_buffer) {
        printf("Failed to allocate frame buffer of size %d bytes\n", state->frame_size);
        return ERR_MEM;
    }
    state->sent_offset = 0;

    // Capture the first frame.
    PicoHM01B0_start_capture(&camera, state->frame_buffer);
    PicoHM01B0_wait_for_frame(&camera);

    // Send the packet (header + first chunk of image data).
    if (send_frame_packet(newpcb, state) != ERR_OK) {
        return ERR_VAL;
    }
    printf("Client connected, streaming frame data... (Resolution: %u x %u, Frame size: %u bytes)\n",
           width, height, state->frame_size);
    return ERR_OK;
}

/* Main server loop running on core1 */
static void telemetry_server_thread(void) {
    telemetry_state = calloc(1, sizeof(telemetry_state_t));
    if (!telemetry_state) {
        printf("Failed to allocate telemetry state\n");
        return;
    }
    telemetry_state->frame_buffer = NULL;
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("Failed to create new TCP PCB\n");
        free(telemetry_state);
        telemetry_state = NULL;
        return;
    }
    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err != ERR_OK) {
        printf("tcp_bind failed: %d\n", err);
        tcp_close(pcb);
        free(telemetry_state);
        telemetry_state = NULL;
        return;
    }
    struct tcp_pcb *listen_pcb = tcp_listen(pcb);
    if (!listen_pcb) {
        printf("tcp_listen failed\n");
        tcp_close(pcb);
        free(telemetry_state);
        telemetry_state = NULL;
        return;
    }
    tcp_arg(listen_pcb, telemetry_state);
    tcp_accept(listen_pcb, telemetry_tcp_accept);
    printf("Telemetry TCP server listening on port %d\n", TCP_PORT);

    // Process lwIP events
    while (1) {
#if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1000));
#else
        sleep_ms(1000);
#endif
    }
}

/* API: Start streaming telemetry data.
   This function launches the telemetry server on core1.
   Wi-Fi connection is established before launching the server. */
int telemetry_stream_start(const char* ssid, const char* password, uint16_t port) {
    (void)port; // Here, TCP_PORT is used as defined.

    // Initialize Wi-Fi on the calling core (which will be used by core1 callbacks).
    if (cyw43_arch_init()) {
        printf("cyw43_arch init failed\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 60000)) {
        printf("WiFi connect failed\n");
        return -1;
    }
    printf("Connected to WiFi\n");

    // Launch the telemetry server on core1.
    multicore_launch_core1(telemetry_server_thread);

    return 0;
}

void telemetry_stream_stop(void) {
    // For simplicity, a graceful shutdown is not implemented.
    printf("Telemetry stream stop not implemented\n");
}
