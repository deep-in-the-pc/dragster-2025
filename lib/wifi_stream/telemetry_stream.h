#ifndef TELEMETRY_STREAM_H
#define TELEMETRY_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Start the telemetry stream over Wi-Fi.
 *
 * This function initializes the Wi-Fi connection using the provided SSID and password,
 * and launches a TCP server on core1 that streams HM01B0 frame data.
 *
 * @param ssid      Wi-Fi network name.
 * @param password  Wi-Fi network password.
 * @param port      TCP port to use (currently a fixed port is used internally).
 * @return 0 on success, -1 on failure.
 */
int telemetry_stream_start(const char* ssid, const char* password, uint16_t port);

/**
 * @brief Stop the telemetry stream.
 */
void telemetry_stream_stop(void);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_STREAM_H
