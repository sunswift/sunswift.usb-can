# Makefile for Sunswift USB-CAN board. 

.PHONY: clean

MAIN_NAME=usbcan
ARCH=msp430
MCU=msp430x149
BUILD_DIR=.
OUTPUT_DIR=../outputs

AS=msp430-as
CC=msp430-gcc
OBJCOPY=msp430-objcopy
OBJDUMP=msp430-objdump

JTAG=msp430-jtag
#JTAG=jtag.py

CFLAGS=-g -O2 -Wall -mmcu=$(MCU) -I../scandal/include -I../mcp2510 -I. -I.. -I../$(MAIN_NAME)
LDFLAGS=-mmcu=$(MCU) 
ASFLAGS=-mmcu=$(MCU) 

all: $(MAIN_NAME).hex $(MAIN_NAME).lst
	cp $(MAIN_NAME).hex $(OUTPUT_DIR)/$(MAIN_NAME).hex

## Copy the .hex and .elf file
 
$(MAIN_NAME).hex: $(MAIN_NAME).elf
	$(OBJCOPY) -O ihex $< $@

$(MAIN_NAME).lst: $(MAIN_NAME).elf
	$(OBJDUMP) -DS $(MAIN_NAME).elf > ${MAIN_NAME}.lst	

## Link the .o files into the .out (.elf) file. 

$(MAIN_NAME).elf :  $(MAIN_NAME).o spi_driver.o led.o scandal_timer.o scandal_obligations.o $(BUILD_DIR)/scandal_eeprom.o $(BUILD_DIR)/mcp2510.o $(BUILD_DIR)/scandal_error.o $(BUILD_DIR)/scandal_engine.o  $(BUILD_DIR)/scandal_utils.o $(BUILD_DIR)/scandal_message.o serial.o
	$(CC) $^ $(LDFLAGS) -o $(MAIN_NAME).elf

## How to build .o, .s and .o files from .c, .c and .s files respectively. 

%.o : %.c 
	$(CC) -c $(CFLAGS) $< -o $@

%.s : %.c
	$(CC) -S $(CFLAGS) $< -o $@

%.o : %.s
	$(AS) -c $(ASFLAGS) $< -o $@

## How to build individual

$(BUILD_DIR)/scandal_eeprom.o : ../mspenv/scandal_eeprom.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/mcp2510.o : ../mcp2510/mcp2510.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/scandal_error.o : ../scandal/source/scandal_error.c
	$(CC) $(CFLAGS) -c -o $@ $<	

$(BUILD_DIR)/scandal_engine.o : ../scandal/source/scandal_engine.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/scandal_utils.o : ../scandal/source/scandal_utils.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/scandal_message.o : ../scandal/source/scandal_message.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f *.o *.out *.map *.hex
	rm *~
	rm -f $(BUILD_DIR)/*.o
	rm $(MAIN_NAME).lst $(MAIN_NAME).elf

program: $(MAIN_NAME).hex
	$(JTAG) -eEpvrw $(MAIN_NAME).hex
