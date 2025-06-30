# Project name
PROJECT = secure_bootloader

# Toolchain
CC       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
GDB      := arm-none-eabi-gdb

# Directories
BUILD_DIR       := build
# Define base source directories
CORE_SRC_DIR    := Core/Src
APP_SRC_DIR     := App/Src
DRIVERS_SRC_DIR := Drivers/Src
MBEDTLS_DIR     := lib/mbedtls
STM32LIB_DIR	:= lib/STM32F767_Includes

# Include Paths 
INC_PATHS := \
	-ICore/Inc	\
	-IApp/Inc/bootloader	\
	-IApp/Inc/cli	\
	-IApp/Inc/ota	\
	-IApp/Inc/common	\
	-IDrivers/Inc/	\
	-I$(STM32LIB_DIR) \
	-I$(MBEDTLS_DIR)/include # mbedTLS include

# Source Files
C_SRCS := $(shell find $(CORE_SRC_DIR) $(APP_SRC_DIR) $(DRIVERS_SRC_DIR) -name "*.c")
OBJS := $(patsubst %.c, $(BUILD_DIR)/%.o, $(C_SRCS))

# MBEDTLS Library Path
MBEDTLS_LIB := $(MBEDTLS_DIR)/build/library/libmbedcrypto.a

# Output Files
TARGET := $(BUILD_DIR)/$(PROJECT).elf
HEX    := $(BUILD_DIR)/$(PROJECT).hex
BIN    := $(BUILD_DIR)/$(PROJECT).bin

# Flags
CFLAGS  := -mcpu=cortex-m7 -mthumb -O0 -g -ggdb -Wall -DSTM32F767xx 
CFLAGS 	+= -ffunction-sections -fdata-sections
CFLAGS 	+= $(INC_PATHS)

# Linker Flags
LDFLAGS := -T stm32f767z.ld -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/secure_bootloader.map
LDFLAGS += -specs=nosys.specs

# Default target
all: $(TARGET)

$(TARGET): $(OBJS) $(MBEDTLS_LIB) | $(BUILD_DIR)
	@echo "--- Linking Final Application ---"
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(MBEDTLS_LIB) -o $@

# Linking Rule
$(MBEDTLS_LIB):
	@echo "--- Building mbedtls library ---"
	$(MAKE) -C $(MBEDTLS_DIR)

# Compile source files
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "Compiling $<..."
	@mkdir -p $(dir $@) # Ensure the subdirectory in build/ exists
	$(CC) $(CFLAGS) -c $< -o $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Clean up
clean:
	@echo "--- Cleaning project files ---"
	rm -rf $(BUILD_DIR)
	@echo "--- Cleaning mbedtls library ---"
	$(MAKE) -C $(MBEDTLS_DIR) clean

# Flash target
flash: $(TARGET)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $< verify reset exit"

# Debug target
debug: $(TARGET)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg & \
	$(GDB) $< -ex "target remote :3333"