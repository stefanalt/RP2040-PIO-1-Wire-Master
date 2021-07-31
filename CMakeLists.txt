cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
#include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)
include(/home/as/pico/pico-sdk/tools/CMakeLists.txt)

project(owtest_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(owtest
 owtest.c
)

pico_generate_pio_header(owtest ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
pico_generate_pio_header(owtest ${CMAKE_CURRENT_LIST_DIR}/onewire.pio)

pico_enable_stdio_usb(owtest 1)
pico_add_extra_outputs(owtest)

#target_sources(pio_blink PRIVATE owtest.c)

target_link_libraries(owtest pico_stdlib hardware_pio)
