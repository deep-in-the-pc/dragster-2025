add_executable(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.c)

target_sources(${NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dragster_bno055.c
)

target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} pico_stdlib pico_binary_info PicoHM01B0 PicoBNO055 TELEMETRY_STREAM pico_cyw43_arch hardware_vreg hardware_clocks hardware_flash hardware_sync) 

set_target_properties(${NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})