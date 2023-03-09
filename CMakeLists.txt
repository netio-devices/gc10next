cmake_minimum_required(VERSION 3.13)

include (pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(main
	main.c ssd1306.c
	)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

add_compile_definitions(PICO_DEFAULT_UART_INSTANCE=uart1)
add_compile_definitions(PICO_DEFAULT_UART_TX_PIN=4)
add_compile_definitions(PICO_DEFAULT_UART_RX_PIN=5)

pico_add_extra_outputs(main)

target_link_libraries(main pico_stdlib hardware_adc hardware_pwm hardware_i2c)
