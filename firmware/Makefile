# Top level makefile for template project
# The only modifications you should need to make are to change the CHIP, MAIN_NAME and
# add any custom C source files to OBJECTS

# Set CHIP to be lpc11c14, lpc1768 or msp430f149 depending on what micro you're using
CHIP = msp430f149

# This should be what your main C file is called
MAIN_NAME = usbcan

# Directory definitions
ROOT = ..
SCANDAL = $(ROOT)/scandal
ARCH = $(SCANDAL)/src/arch/$(CHIP)
DRIVERS = $(ARCH)/drivers
UTILS = $(ROOT)/utilities
BUILD = ./build# never . or clean will delete everything
SRC = ./src

# Custom project objects
OBJECTS  = $(MAIN_NAME).o scandal_obligations.o
# Scandal objects
OBJECTS += engine.o error.o manager.o maths.o message.o utils.o stdio.o leds.o wavesculptor.o
# Driver objects
OBJECTS += can.o flash.o uart.o gpio.o timer.o wdt.o system.o
# Add other objects here or in the architecture specific makefile
OBJECTS +=

CFLAGS  = -I$(SCANDAL)/include # for scandal includes
CFLAGS += -I$(ARCH)/include # for arch drivers
CFLAGS += -I$(ARCH) #for anything else
CFLAGS += -I./include # for project local includes
# We should really try to minimise all warnings for error free code. Wall is needed for this.
CFLAGS += -Wall
# Optimise for size. Can also optimise for performance with 1, 2 or 3
CFLAGS += -Os
CFLAGS += -D$(CHIP)

.PHONY: clean realclean cscope

# For printing out colours in makefile scripts
RED="\\033[91m"
NORMAL="\\033[0m"
YELLOW="\\e[1\;33m"

# Include the chip specific makefile
include makefile.$(CHIP)

# This target is special because it's the first in the list. We should make the build directory
# here and do anything else that only needs to happen once
$(BUILD)/$(MAIN_NAME).o: src/$(MAIN_NAME).c
	@if [ ! -e "cscope.out" ]; then \
		echo -e "[HINT] $(RED)You should learn to use cscope! Type make cscope$(NORMAL)"; \
	fi
	@mkdir -p $(BUILD)
	@echo "[CC] $@"
	@if [ "$(BUILD)" == "." ]; then echo -e "$(RED)Don't use . as BUILD$(NORMAL)"; exit 1; fi
	@$(CC) $(CFLAGS) -c -o $@ $<

# Project local objects
$(BUILD)/%.o: src/%.c
	@echo "[CC] $@"
	@$(CC) $(CFLAGS) -c -o $@ $<

# Architecture specific drivers
$(BUILD)/%.o: $(DRIVERS)/%.c
	@echo "[CC] $@"
	@$(CC) $(CFLAGS) -c -o $@ $<

# Scandal engine
$(BUILD)/%.o: $(SCANDAL)/src/%.c
	@echo "[CC] $@"
	@$(CC) $(CFLAGS) -c -o $@ $<

# Scandal drivers
$(BUILD)/%.o: $(SCANDAL)/src/drivers/%.c
	@echo "[CC] $@"
	@$(CC) $(CFLAGS) -c -o $@ $<

# Remove the build directory
clean:
	@echo "[CLEAN] $(BUILD)"
	@if [ "$(BUILD)" == "." ]; then echo -e "$(RED)Don't use . as BUILD$(NORMAL)"; exit 1; fi
	@rm -Rf $(BUILD)

# Remove build directory and cscope
realclean: clean
	@rm -f cscope.out cscope.files

# Generate cscope database from all c, h and assembly files
cscope:
	@echo "[GEN] cscope"
	@if [ -d "$(SCANDAL)" ]; then \
		find $(SCANDAL) -name "*.[chSs]" >> cscope.files; \
	fi
	@find . -name "*.[chSs]">> cscope.files
	@cscope -b -f cscope.out
