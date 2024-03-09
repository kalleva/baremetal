TARGET = STM32L411CEU6-SPI-MASTER-PING
BUILD_DIR = build

CC=arm-none-eabi-gcc
OC=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
OS=arm-none-eabi-size

MCU=STM32L411CE
MCU_SPEC=cortex-m4
LINKER_SCRIPT=stm32l411ceu6.ld

CFLAGS += -c
CFLAGS += -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
CFLAGS += -mhard-float
CFLAGS += -mfloat-abi=hard
CFLAGS += -mfpu=fpv4-sp-d16
CFLAGS += -Wall
CFLAGS += -fmessage-length=0
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -fno-builtin
CFLAGS += -g
CFLAGS += -Os
CFLAGS += -nostdlib

LDFLAGS += -mcpu=$(MCU_SPEC)
LDFLAGS += -mthumb
LDFLAGS += -mhard-float
LDFLAGS += -mfloat-abi=hard
LDFLAGS += -mfpu=fpv4-sp-d16
LDFLAGS += -nostdlib
LDFLAGS += -nodefaultlibs
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -T$(LINKER_SCRIPT)

INCLUDE += -I.
INCLUDE += -I./cmsis_headers

DEFINE += -DSTM32F411xE

C_SRC += ./stm32l411ceu6_startup.c
C_SRC += ./clock.c
C_SRC += ./led.c
C_SRC += ./uart.c
C_SRC += ./spi.c
C_SRC += ./system.c
C_SRC += ./main.c

OBJS += $(C_SRC:.c=.o)

.PHONY: all
all: $(TARGET).bin $(TARGET).hex

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) $(DEFINE) -o $@ $^

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OC) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

.PHONY: clean
clean:
	del /Q /F *.o *.elf *.map *.bin *.hex

.PHONY: update_lsp
lsp:
	compiledb -n make

.PHONY: flash
flash:
	openocd -f board/st_nucleo_f4.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

.PHONY: load
load:
	openocd -f board/st_nucleo_f4.cfg