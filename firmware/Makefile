# Makefile for Sunswift USB-CAN board. 

.PHONY: clean

MAIN_NAME=usbcan
ARCH=msp430
MCU=msp430x149
BUILD=./build

SCANDAL=../scandal
ARCH=$(SCANDAL)/src/arch/msp430f149

AS=msp430-as
CC=msp430-gcc
OBJCOPY=msp430-objcopy
OBJDUMP=msp430-objdump

JTAG=/opt/mspgcc/bin/msp430-jtag

CFLAGS=-g -O1 -Wall -mmcu=$(MCU) -I$(SCANDAL)/include -I$(ARCH)/include -I./include
LDFLAGS=-mmcu=$(MCU) 
ASFLAGS=-mmcu=$(MCU) 

OBJECTS=$(MAIN_NAME).o spi_driver.o led.o scandal_timer.o scandal_obligations.o flash.o mcp2510.o error.o engine.o utils.o message.o serial.o maths.o

all: $(MAIN_NAME).hex $(MAIN_NAME).lst

## Copy the .hex and .elf file
$(MAIN_NAME).hex: $(MAIN_NAME).elf
	$(OBJCOPY) -O ihex $(BUILD)/$< $(BUILD)/$@

$(MAIN_NAME).lst: $(MAIN_NAME).elf
	$(OBJDUMP) -DS $(BUILD)/$(MAIN_NAME).elf > $(BUILD)/${MAIN_NAME}.lst

## Link the .o files into the .out (.elf) file. 

$(MAIN_NAME).elf: $(OBJECTS)
	@mkdir -p $(BUILD)
	$(CC)  $(addprefix $(BUILD)/,$(OBJECTS)) $(LDFLAGS) -o $(BUILD)/$(MAIN_NAME).elf

## How to build .o, .s and .o files from .c, .c and .s files respectively. 

%.o: src/%.c 
	@mkdir -p $(BUILD)
	$(CC) -c $(CFLAGS) $< -o $(BUILD)/$@

%.s: %.c
	$(CC) -S $(CFLAGS) $< -o $(BUILD)/$@

%.o: %.s
	$(AS) -c $(ASFLAGS) $< -o $(BUILD)/$@

## How to build individual

flash.o: $(ARCH)/drivers/flash.c
	$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

mcp2510.o: $(ARCH)/drivers/mcp2510.c
	$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

error.o: $(SCANDAL)/src/error.c
	$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

engine.o: $(SCANDAL)/src/engine.c
	$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

maths.o: $(SCANDAL)/src/maths.c
	$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

utils.o: $(SCANDAL)/src/utils.c
	$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

message.o: $(SCANDAL)/src/message.c
	$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

clean:
	rm -Rf $(BUILD)

realclean: clean
	rm cscope.files cscope.out

cscope:
	@echo "[GEN] cscope"
	@if [ -d "$(SCANDAL)" ]; then \
		find $(SCANDAL) -name "*.[chSs]" >> cscope.files; \
	fi
	@find . -name "*.[chSs]">> cscope.files
	@cscope -b -f cscope.out

program: $(MAIN_NAME).hex
	@scp $(BUILD)/$(MAIN_NAME).hex yotsuba:/tmp;
	@ssh yotsuba $(JTAG) -eEpvrw /tmp/$(MAIN_NAME).hex
	#$(JTAG) -eEpvrw $(BUILD)/$(MAIN_NAME).hex

reset:
	@ssh yotsuba $(JTAG) -r
	#$(JTAG) -r
