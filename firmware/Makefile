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

CFLAGS=-g -O1 -Wall -mmcu=$(MCU) -I$(SCANDAL)/include -I$(ARCH_DIR)/include -I./include
LDFLAGS=-mmcu=$(MCU) 
ASFLAGS=-mmcu=$(MCU) 

all: $(MAIN_NAME).hex $(MAIN_NAME).lst

## Copy the .hex and .elf file
$(MAIN_NAME).hex: $(MAIN_NAME).elf
	$(OBJCOPY) -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

$(MAIN_NAME).lst: $(MAIN_NAME).elf
	$(OBJDUMP) -DS $(BUILD_DIR)/$(MAIN_NAME).elf > $(BUILD_DIR)/${MAIN_NAME}.lst

## Link the .o files into the .out (.elf) file. 

$(MAIN_NAME).elf :  $(BUILD_DIR)/$(MAIN_NAME).o $(BUILD_DIR)/spi_driver.o $(BUILD_DIR)/led.o $(BUILD_DIR)/scandal_timer.o $(BUILD_DIR)/scandal_obligations.o $(BUILD_DIR)/flash.o $(BUILD_DIR)/mcp2510.o $(BUILD_DIR)/error.o $(BUILD_DIR)/engine.o  $(BUILD_DIR)/utils.o $(BUILD_DIR)/message.o $(BUILD_DIR)/serial.o $(BUILD_DIR)/maths.o
	@mkdir -p $(BUILD_DIR)
	$(CC) $^ $(LDFLAGS) -o $(BUILD_DIR)/$(MAIN_NAME).elf

## How to build .o, .s and .o files from .c, .c and .s files respectively. 

$(BUILD_DIR)/%.o : src/%.c 
	@mkdir -p $(BUILD_DIR)
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
	rm -Rf $(BUILD_DIR)

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
	$(JTAG) -eEpvrw $(BUILD_DIR)/$(MAIN_NAME).hex
