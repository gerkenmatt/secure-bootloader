# Makefile for STM32F767 Uart Echo App (A/B and Standard App Build)

# Toolchain settings
CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
GDB     := arm-none-eabi-gdb

# Directories
BUILD_DIR  := build
SRC_DIR    := src
INCLUDE_DIRS := \
  -I./include \
  -ISTM32F767_Includes

# Compilation and Linker Flags 
CFLAGS  := -mcpu=cortex-m7 -mthumb -O1 -g -ggdb -Wall -DSTM32F767xx $(INCLUDE_DIRS)
LDFLAGS := -T stm32f767z.ld -Wl,--gc-sections

# Source and Object Files
SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))

# Project Name
PROJECT := uart_print

# --- Define final binary and ELF files ---
BIN_APP = $(BUILD_DIR)/$(PROJECT)_app.bin
ELF_APP = $(BUILD_DIR)/$(PROJECT)_app.elf

BIN_SLOTA = $(BUILD_DIR)/$(PROJECT)_slota.bin
ELF_SLOTA = $(BUILD_DIR)/$(PROJECT)_slota.elf

BIN_SLOTB = $(BUILD_DIR)/$(PROJECT)_slotb.bin
ELF_SLOTB = $(BUILD_DIR)/$(PROJECT)_slotb.elf
# --- Build Targets ---

# Default target 'all' now builds all three versions
all: $(BIN_APP) $(BIN_SLOTA) $(BIN_SLOTB)

# Specific target to only build the standard application
app: $(BIN_APP)

# --- Binary Creation Rules ---
$(BIN_APP): $(ELF_APP)
	@echo "Creating binary for Standard App..."
	$(OBJCOPY) -O binary $< $@

$(BIN_SLOTA): $(ELF_SLOTA)
	@echo "Creating binary for Slot A..."
	$(OBJCOPY) -O binary $< $@

$(BIN_SLOTB): $(ELF_SLOTB)
	@echo "Creating binary for Slot B..."
	$(OBJCOPY) -O binary $< $@

# --- ELF Linking Rules ---

# Rule to link the ELF file for the Standard App
$(ELF_APP): $(OBJS) | $(BUILD_DIR)
	@echo "--- Linking for Standard App (0x08000000) ---"
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$(PROJECT)_app.map \
	    -Wl,--defsym=__FLASH_ORIGIN=0x08000000 \
	    -Wl,--defsym=__FLASH_LENGTH=256K \
	    -o $@

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

# --- Generic Rules ---

# Compilation Rule
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create Build Directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Clean Rule
clean:
	rm -rf $(BUILD_DIR)

# --- Flash Rules ---

# Flash the Standard App
flash_app: $(BIN_APP)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $(BIN_APP) verify 0x08000000 reset exit"

# Flash Slot A
flash_slota: $(BIN_SLOTA)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $(BIN_SLOTA) verify 0x08040000 reset exit"

# Flash Slot B
flash_slotb: $(BIN_SLOTB)
	openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program $(BIN_SLOTB) verify 0x08100000 reset exit"

# Mark targets that are not files
.PHONY: all app clean flash_app flash_slota flash_slotb