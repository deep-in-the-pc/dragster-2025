add_library(PicoBNO055 INTERFACE)

target_sources(PicoBNO055 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/PicoBNO055.c
    ${CMAKE_CURRENT_LIST_DIR}/PicoBNO055_support.c
)

target_include_directories(PicoBNO055 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(PicoBNO055 INTERFACE hardware_gpio hardware_i2c )