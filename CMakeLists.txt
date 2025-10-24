cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(Multicore C CXX ASM)
pico_sdk_init()

add_executable(${PROJECT_NAME} Multicore.c 
                            lib/bh1750_light_sensor.c 
                            lib/gy33.c 
                            lib/ssd1306.c 
                            lib/matriz_leds.c 
                            lib/led_rgb.c
)

pico_set_program_name(${PROJECT_NAME} "Multicore")
pico_set_program_version(${PROJECT_NAME} "0.1")

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 0)

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        hardware_i2c
        hardware_pio
        hardware_pwm
        hardware_adc
        hardware_uart
        pico_multicore
)
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/lib
)
pico_add_extra_outputs(${PROJECT_NAME})

