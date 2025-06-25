# cmake/arm-none-eabi.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm) # It's good practice to also set the processor

# Ensure CMake uses the correct full path to your compiler if it's not in your system PATH,
# or ensure /opt/gcc-arm-none-eabi/bin (based on your previous log) is in your PATH.
# If it's found via PATH, 'arm-none-eabi-gcc' is fine.
# Example with full path:
# set(CMAKE_C_COMPILER   /opt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc)
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_AR           arm-none-eabi-ar)
set(CMAKE_RANLIB       arm-none-eabi-ranlib)
# Optional, but good to define for completeness if your projects ever use C++ or ASM:
# set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
# set(CMAKE_ASM_COMPILER arm-none-eabi-as)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_OBJDUMP      arm-none-eabi-objdump CACHE INTERNAL "objdump tool")


# Cortex-M7 flags - these are primarily compile flags
set(MCU_COMPILE_FLAGS "-mcpu=cortex-m7 -mthumb -Os -g")
set(CMAKE_C_FLAGS "${MCU_COMPILE_FLAGS}" CACHE STRING "Common C flags for target")
# If you had C++ files:
# set(CMAKE_CXX_FLAGS "${MCU_COMPILE_FLAGS}" CACHE STRING "Common CXX flags for target")

# Linker flags for executables (this includes CMake's test executable)
# Add -specs=nosys.specs to provide stubs for syscalls like _exit.
# Also pass MCU flags to the linker via the compiler driver.
set(CMAKE_EXE_LINKER_FLAGS "-specs=nosys.specs ${MCU_COMPILE_FLAGS}" CACHE STRING "Linker flags for executables")

# For building static libraries, specific linker flags are less critical,
# but ensuring consistency if any internal linking happens is good.
# The -specs=nosys.specs is mostly for CMake's initial test program linking.

# Tell CMake that the compiler is a cross-compiler for a generic system
# This avoids some host system checks
set(CMAKE_CROSSCOMPILING TRUE)

# Optional: Skip rpath handling for embedded targets
set(CMAKE_SKIP_RPATH TRUE)