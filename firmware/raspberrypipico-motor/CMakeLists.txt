cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
include(pico_sdk_import.cmake)
pico_sdk_init()

project(stepper_control C CXX ASM)

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(INC_DIR ${SRC_DIR})
file(GLOB SOURCE_PICO ${SRC_DIR}/pico/*.c)
set(SOURCES
    ${SRC_DIR}/main.c
    ${SOURCE_PICO}
)

add_compile_options(-Wall)
add_executable(stepper_control ${SOURCES})
target_include_directories(stepper_control PRIVATE ${INC_DIR})
target_link_libraries(stepper_control pico_stdlib pico_multicore pico_stdio_usb)
pico_add_extra_outputs(stepper_control)
