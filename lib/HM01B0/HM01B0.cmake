
# Add library cpp files
add_library(HM01B0 INTERFACE)

target_sources(HM01B0 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/hm01b0_driver.c
)

# Add include directory
target_include_directories(HM01B0 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
target_link_libraries(HM01B0 INTERFACE cmsis_core pico_stdlib hardware_i2c hardware_pio hardware_dma hardware_pwm)