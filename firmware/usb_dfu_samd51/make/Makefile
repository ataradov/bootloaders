##############################################################################
BUILD = build
BIN = bl_dfu_d51

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

ifeq ($(OS), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif

CFLAGS += -W -Wall --std=gnu11 -Os
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m4 -mthumb
CFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d
CFLAGS += -static -ffreestanding -nostartfiles
CFLAGS += -flto

LDFLAGS += -mcpu=cortex-m4 -mthumb
LDFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=../linker/samd51j20a.ld
LDFLAGS += -flto -nostdlib

INCLUDES += \
  -I../include \
  -I..

SRCS += \
  ../main.c \
  ../usb.c \
  ../usb_std.c \
  ../usb_dfu.c \
  ../usb_descriptors.c \
  ../startup.c \

DEFINES += \
  -D__SAMD51J20A__ \
  -DDONT_USE_CMSIS_INIT \
  -DF_CPU=48000000

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).dfu size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

$(BUILD)/$(BIN).dfu: $(BUILD)/$(BIN).bin
	@echo DFU $^
	@cp $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).dfu
	@dfu-suffix -a $(BUILD)/$(BIN).dfu -v ffff -p ffff -d ffff

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

