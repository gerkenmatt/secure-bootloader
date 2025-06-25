cmake_minimum_required(VERSION 3.16)

# Set output options for cleaner make output
set(CMAKE_VERBOSE_MAKEFILE OFF CACHE BOOL "Disable verbose make output")
set(CMAKE_COLOR_MAKEFILE ON CACHE BOOL "Enable colored make output")

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/toolchain-arm-none-eabi.cmake" CACHE STRING "Toolchain file")
endif()

# Find required tools
find_program(ARM_OBJCOPY arm-none-eabi-objcopy PATHS /opt/gcc-arm-none-eabi/bin REQUIRED)
find_program(ARM_SIZE arm-none-eabi-size PATHS /opt/gcc-arm-none-eabi/bin REQUIRED)

project(blinky C ASM)

# Configure make to use a progress display
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
set_property(GLOBAL PROPERTY TARGET_MESSAGES OFF)

# MCU Flags
set(CPU_FLAGS "-mcpu=cortex-m7 -mthumb")
set(COMMON_FLAGS "${CPU_FLAGS} -Wall -O2 -g -DSTM32F767xx")
set(CMAKE_C_FLAGS "${COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS}")

# Linker Script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/stm32f767z.ld)

# STM32CubeF7 Path
set(STM32CUBE_F7_PATH "$ENV{HOME}/STM32CubeF7" CACHE PATH "Path to STM32CubeF7")
if(NOT EXISTS ${STM32CUBE_F7_PATH})
    message(FATAL_ERROR "STM32CubeF7 not found at ${STM32CUBE_F7_PATH}. Please set STM32CUBE_F7_PATH to the correct location.")
endif()

# Include directories
include_directories(
    ${STM32CUBE_F7_PATH}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ${STM32CUBE_F7_PATH}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
)

# Source files
set(SOURCES
    src/main.c
    src/startup.c
    src/led_control.c
    src/utils.c
    ${STM32CUBE_F7_PATH}/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
)

# Main executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -specs=nosys.specs
    -specs=nano.specs
    -Wl,-Map=${PROJECT_NAME}.map
)

# Output formats
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${ARM_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${ARM_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${ARM_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)

# Flash target
add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f7x.cfg
                    -c "program $<TARGET_FILE:${PROJECT_NAME}.elf> verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf
)

# Debug target
add_custom_target(debug
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f7x.cfg &
    COMMAND arm-none-eabi-gdb $<TARGET_FILE:${PROJECT_NAME}.elf> -ex "target remote :3333"
)
