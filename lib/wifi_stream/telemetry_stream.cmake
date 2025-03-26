# telemetry_stream.cmake

# Create a INTERFACE library from telemetry_stream.c.
add_library(TELEMETRY_STREAM STATIC
    ${CMAKE_CURRENT_LIST_DIR}/telemetry_stream.c
)

# Add the current directory (where telemetry_stream.h is located) to the include path.
target_include_directories(TELEMETRY_STREAM PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../src
)

# Link any necessary libraries.
# For example, if your telemetry_stream code uses pico_stdlib and the Pico W's cyw43_arch,
# you might add:
target_link_libraries(TELEMETRY_STREAM PUBLIC
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_multicore
    PicoHM01B0
)
