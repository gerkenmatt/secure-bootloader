
# Project name
PROJECT = secure_bootloader

# Toolchain
CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
GDB     := arm-none-eabi-gdb

# Directories
BUILD_DIR  := build
SRC_DIR    := src
INCLUDE_DIRS := \
  -Iinclude \
  -I$(HOME)/STM32CubeF7/Drivers/STM32F7xx_HAL_Driver/Inc \
  -I$(HOME)/STM32CubeF7/Drivers/CMSIS/Device/ST/STM32F7xx/Include \
  -I$(HOME)/STM32CubeF7/Drivers/CMSIS/Include
MBEDTLS_DIR := lib/mbedtls

# Source Files        
SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))


# Output Files
TARGET := $(BUILD_DIR)/$(PROJECT).elf
HEX    := $(BUILD_DIR)/$(PROJECT).hex
BIN    := $(BUILD_DIR)/$(PROJECT).bin

# Flags
CFLAGS  := -mcpu=cortex-m7 -mthumb -Os -g -ggdb -Wall -DSTM32F767xx $(INCLUDE_DIRS)
CFLAGS  += -I$(MBEDTLS_DIR)/include
CFLAGS  += -ffunction-sections -fdata-sections
LDFLAGS := -T stm32f767z.ld -Wl,--gc-sections
LDFLAGS += lib/mbedtls/build/library/libmbedcrypto.a

# Default target
all: $(TARGET)

# Linking
$(TARGET): $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@
	$(OBJCOPY) -O ihex $@ $(HEX)
	$(OBJCOPY) -O binary $@ $(BIN)

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Clean up
clean:
	rm -rf $(BUILD_DIR)

# Flash target
flash: $(TARGET)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $< verify reset exit"

# Debug target
debug: $(TARGET)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg & \
	$(GDB) $< -ex "target remote :3333"
