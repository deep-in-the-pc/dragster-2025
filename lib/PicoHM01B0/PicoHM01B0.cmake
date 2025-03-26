
# Add library cpp files
add_library(PicoHM01B0 INTERFACE)

target_sources(PicoHM01B0 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/PicoHM01B0.c
)

# Add include directory
target_include_directories(PicoHM01B0 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
target_link_libraries(PicoHM01B0 INTERFACE cmsis_core pico_stdlib hardware_i2c hardware_pio hardware_dma hardware_clocks pico_time)