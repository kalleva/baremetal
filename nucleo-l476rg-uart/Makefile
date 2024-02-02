TARGET = NUCLEO-L476RG-UART

CC=arm-none-eabi-gcc
OC=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
OS=arm-none-eabi-size

MCU=STM32L476RG
MCU_SPEC=cortex-m4
LINKER_SCRIPT=nucleo_l476rg_ls.ld

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
CFLAGS += -g
CFLAGS += -O0
CFLAGS += -nostdlib

LDFLAGS += -mcpu=$(MCU_SPEC)
LDFLAGS += -mthumb
LDFLAGS += -mhard-float
LDFLAGS += -mfloat-abi=hard
LDFLAGS += -mfpu=fpv4-sp-d16
LDFLAGS += -nostdlib
LDFLAGS += -lgcc
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -T$(LINKER_SCRIPT)

INCLUDE += -I.
INCLUDE += -I./cmsis_headers

DEFINE += -DSTM32L476xx

C_SRC += ./nucleo_l476rg_startup.c
C_SRC += ./main.c

OBJS += $(C_SRC:.c=.o)

.PHONY: all
all:$(TARGET).bin $(TARGET).hex

%.o:%.c
	$(CC) $(CFLAGS) $(INCLUDE) $(DEFINE) -o $@ $^

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

$(TARGET).hex:$(TARGET).elf
	$(OC) -O ihex $< $@

$(TARGET).bin:$(TARGET).elf
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
	openocd -f board/st_nucleo_l4.cfg -c "program $(TARGET).elf verify reset exit"

.PHONY: load
load:
	openocd -f board/st_nucleo_l4.cfg