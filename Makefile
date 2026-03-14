# ================================================================
# Shell
# ================================================================
SHELL := /bin/bash

# ================================================================
# Toolchain
# ================================================================
CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size

# ================================================================
# Project
# ================================================================
TARGET := fcm
BUILD  := build

SRC_DIRS := app arch bsp lib/src os mod

# ================================================================
# MCU
# ================================================================
MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# ================================================================
# Flags
# ================================================================
CFLAGS := $(MCU) -O0 -Wall -g \
          -ffunction-sections -fdata-sections \
          -MMD -MP \
          -DUSE_STDPERIPH_DRIVER \
          -DSTM32F40_41xxx \
	  -DARM_MATH_CM4 \
	  -D__FPU_PRESENT=1 \
	  -Iapp \
          -Iarch \
	  -Iarch/SupportFunctions \
	  -Iarch/MatrixFunctions \
          -Ibsp \
          -Imain \
          -Ilib/inc \
	  -Ios \
	  -Imod

ASFLAGS := $(MCU) -x assembler-with-cpp -g

LDFLAGS := $(MCU) \
           -T ld/stm32_flash.ld \
           -Wl,--gc-sections \
           -Wl,-Map=$(BUILD)/$(TARGET).map \
           --specs=nano.specs \
           -u _printf_float \
	   -lm


# ================================================================
# Source scan
# ================================================================
C_SRCS := $(shell find $(SRC_DIRS) -name "*.c")
S_SRCS := $(shell find $(SRC_DIRS) -name "*.s")

OBJS := $(addprefix $(BUILD)/,$(C_SRCS:.c=.o))
OBJS += $(addprefix $(BUILD)/,$(S_SRCS:.s=.o))

DEPS := $(OBJS:.o=.d)

# ================================================================
# Default target
# ================================================================
all: $(BUILD)/$(TARGET).bin

# ================================================================
# Compile C
# ================================================================
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# ================================================================
# Compile ASM (.S)
# ================================================================
$(BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

# ================================================================
# Compile ASM (.s)
# ================================================================
$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

# ================================================================
# Link
# ================================================================
$(BUILD)/$(TARGET).elf: $(OBJS)
	@echo "LD  $@"
	@$(CC) $(OBJS) $(LDFLAGS) -o $@
	@$(SIZE) $@

# ================================================================
# Generate binary + listing
# ================================================================
$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O binary $< $@
	@$(OBJDUMP) -d $< > $(BUILD)/$(TARGET).lst
	@echo "Build complete: $@"

# ================================================================
# Clean
# ================================================================
clean:
	rm -rf $(BUILD)

.PHONY: all clean

# ================================================================
# Auto dependency include
# ================================================================
-include $(DEPS)
