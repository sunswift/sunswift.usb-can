# Makefile for Sunswift USB-CAN board. 

.PHONY: clean

MAIN_NAME=usbcan
ARCH=msp430
MCU=msp430x149
BUILD_DIR=./build

SCANDAL=../scandal
ARCH_DIR=$(SCANDAL)/src/arch/msp430f149

AS=msp430-as
CC=msp430-gcc
OBJCOPY=msp430-objcopy
OBJDUMP=msp430-objdump

JTAG=/opt/mspgcc/bin/msp430-jtag

CFLAGS=-g -O2 -Wall -mmcu=$(MCU) -I$(SCANDAL)/include -I$(ARCH_DIR)/include -I./include
LDFLAGS=-mmcu=$(MCU) 
ASFLAGS=-mmcu=$(MCU) 

all: $(MAIN_NAME).hex $(MAIN_NAME).lst
	cp $(MAIN_NAME).hex $(BUILD_DIR)/$(MAIN_NAME).hex

## Copy the .hex and .elf file
$(MAIN_NAME).hex: $(MAIN_NAME).elf
	$(OBJCOPY) -O ihex $< $@

$(MAIN_NAME).lst: $(MAIN_NAME).elf
	$(OBJDUMP) -DS $(MAIN_NAME).elf > ${MAIN_NAME}.lst	

## Link the .o files into the .out (.elf) file. 

$(MAIN_NAME).elf :  $(BUILD_DIR)/$(MAIN_NAME).o $(BUILD_DIR)/spi_driver.o $(BUILD_DIR)/led.o $(BUILD_DIR)/scandal_timer.o $(BUILD_DIR)/scandal_obligations.o $(BUILD_DIR)/flash.o $(BUILD_DIR)/mcp2510.o $(BUILD_DIR)/error.o $(BUILD_DIR)/engine.o  $(BUILD_DIR)/utils.o $(BUILD_DIR)/message.o $(BUILD_DIR)/serial.o $(BUILD_DIR)/maths.o
	$(CC) $^ $(LDFLAGS) -o $(MAIN_NAME).elf

## How to build .o, .s and .o files from .c, .c and .s files respectively. 

$(BUILD_DIR)/%.o : src/%.c 
	$(CC) -c $(CFLAGS) $< -o $@

%.s : %.c
	$(CC) -S $(CFLAGS) $< -o $@

%.o : %.s
	$(AS) -c $(ASFLAGS) $< -o $@

## How to build individual

$(BUILD_DIR)/flash.o : $(ARCH_DIR)/drivers/flash.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/mcp2510.o : $(ARCH_DIR)/drivers/mcp2510.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/error.o : $(SCANDAL)/src/error.c
	$(CC) $(CFLAGS) -c -o $@ $<	

$(BUILD_DIR)/engine.o : $(SCANDAL)/src/engine.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/maths.o : $(SCANDAL)/src/maths.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/utils.o : $(SCANDAL)/src/utils.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/message.o : $(SCANDAL)/src/message.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f *.o *.out *.map *.hex
	rm *~
	rm -f $(BUILD_DIR)/*.o
	rm $(MAIN_NAME).lst $(MAIN_NAME).elf

program: $(MAIN_NAME).hex
	$(JTAG) -eEpvrw $(MAIN_NAME).hex
