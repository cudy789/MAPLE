cmake_minimum_required(VERSION 3.13)
include(pico-sdk/pico_sdk_init.cmake)

project(firmware)
pico_sdk_init()

add_executable(firmware
	driver_pico-w.c
	main.c
	mongoose.c
	net.c
	packed_fs.c
)

target_include_directories(firmware PUBLIC
  .
)

target_link_libraries(firmware hardware_pio hardware_dma pico_stdlib pico_rand cyw43_driver_picow)
pico_add_extra_outputs(firmware)     # create map/bin/hex file etc.

pico_enable_stdio_usb(firmware 1)    # Route stdio
pico_enable_stdio_uart(firmware 0)   # to USB

# Mongoose build flags in mongoose_config.h

# Example build options
add_definitions(-DHTTP_URL="http://0.0.0.0/")

