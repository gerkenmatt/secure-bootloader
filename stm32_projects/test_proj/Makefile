# Makefile for STM32F767 Blinky Project

# Toolchain settings
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
GDB = arm-none-eabi-gdb

# Directories
BUILD_DIR = build
SRC_DIR = .
INCLUDE_DIRS = -I$(HOME)/STM32CubeF7/Drivers/STM32F7xx_HAL_Driver/Inc \
               -I$(HOME)/STM32CubeF7/Drivers/CMSIS/Device/ST/STM32F7xx/Include \
               -I$(HOME)/STM32CubeF7/Drivers/CMSIS/Include

# Flash address and size (default: bootloader)
FLASH_ADDR ?= 0x08000000
FLASH_LEN  ?= 1024K

ifeq ($(MAKECMDGOALS),flash_app)
FLASH_ADDR := 0x08040000
FLASH_LEN  := 512K
else
FLASH_ADDR ?= 0x08000000
FLASH_LEN  ?= 1024K
endif


# Compilation flags
CFLAGS  := -mcpu=cortex-m7 -mthumb -O0 -g -ggdb -Wall -DSTM32F767xx -DFLASH_ADDR=$(FLASH_ADDR) $(INCLUDE_DIRS)
LDFLAGS := -T stm32f767z.ld -Wl,--gc-sections -Wl,--defsym=__FLASH_ORIGIN=$(FLASH_ADDR) -Wl,--defsym=__FLASH_LENGTH=$(FLASH_LEN)


# Source and Object Files
SRCS = main.c startup.c
OBJS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(SRCS))
TARGET = $(BUILD_DIR)/blinky.elf
HEX = $(BUILD_DIR)/blinky.hex
BIN = $(BUILD_DIR)/blinky.bin

# Default Target
all: $(TARGET)

# Linking
$(TARGET): $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@
	$(OBJCOPY) -O ihex $@ $(HEX)
	$(OBJCOPY) -O binary $@ $(BIN)

# Compilation
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Ensure build directory exists
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Flash at application address
flash_app: $(TARGET)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $(TARGET) verify reset exit"


# Flash using OpenOCD
flash: $(TARGET)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $(TARGET) verify reset exit"

# Debug using OpenOCD
debug: $(TARGET)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg & \
	$(GDB) $(TARGET) -ex "target remote :3333"
