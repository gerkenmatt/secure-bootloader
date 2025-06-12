# Makefile for STM32F767 Blinky Project (A/B Binary Build)

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

# --- Compilation and Linker Flags ---
CFLAGS  := -mcpu=cortex-m7 -mthumb -O1 -g -ggdb -Wall -DSTM32F767xx $(INCLUDE_DIRS)
LDFLAGS := -T stm32f767z.ld -Wl,--gc-sections

# Source and Object Files
SRCS = main.c startup.c 
OBJS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(SRCS))

# Project name
PROJECT = blinky

# --- Build Targets ---
# Define the final binary and ELF files using the project name
BIN_SLOTA = $(BUILD_DIR)/$(PROJECT)_slota.bin
BIN_SLOTB = $(BUILD_DIR)/$(PROJECT)_slotb.bin
ELF_SLOTA = $(BUILD_DIR)/$(PROJECT)_slota.elf
ELF_SLOTB = $(BUILD_DIR)/$(PROJECT)_slotb.elf

# Default target builds both versions
all: $(BIN_SLOTA) $(BIN_SLOTB)

# Rule to create Slot A binary from its specific ELF file
$(BIN_SLOTA): $(ELF_SLOTA)
	@echo "Creating binary for Slot A..."
	$(OBJCOPY) -O binary $< $@

# Rule to create Slot B binary from its specific ELF file
$(BIN_SLOTB): $(ELF_SLOTB)
	@echo "Creating binary for Slot B..."
	$(OBJCOPY) -O binary $< $@

# Rule to link the ELF file for Slot A
$(ELF_SLOTA): $(OBJS) | $(BUILD_DIR)
	@echo "--- Linking for Slot A (0x08040000) ---"
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$(PROJECT)_slota.map \
	    -Wl,--defsym=__FLASH_ORIGIN=0x08040000 \
	    -Wl,--defsym=__FLASH_LENGTH=768K \
	    -o $@

# Rule to link the ELF file for Slot B
$(ELF_SLOTB): $(OBJS) | $(BUILD_DIR)
	@echo "--- Linking for Slot B (0x08100000) ---"
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$(PROJECT)_slotb.map \
	    -Wl,--defsym=__FLASH_ORIGIN=0x08100000 \
	    -Wl,--defsym=__FLASH_LENGTH=768K \
	    -o $@

# Generic Compilation Rule
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Utility targets
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

# Flash Slot A for development/testing
flash_slota: $(BIN_SLOTA)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $(BIN_SLOTA) verify 0x08040000 reset exit"

# Flash Slot B for development/testing
flash_slotb: $(BIN_SLOTB)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $(BIN_SLOTB) verify 0x08100000 reset exit"

# Mark targets that are not files to prevent confusion if a file with that name exists
.PHONY: all clean flash_slota flash_slotb

