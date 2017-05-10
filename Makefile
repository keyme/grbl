#  Part of Grbl
#
#  Copyright (c) 2009-2011 Simen Svale Skogsrud
#  Copyright (c) 2012 Sungeun K. Jeon
#
#  Grbl is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Grbl is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.


# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

VERSION = 0.3.12

DEVICE     ?= atmega2560
CLOCK      = 16000000
PORT       ?= /dev/ttyUSB0
PROGRAMMER ?= -c wiring -P $(PORT) -v -v
OBJECTS    = main.o motion_control.o gcode.o spindle_control.o serial.o \
             protocol.o stepper.o eeprom.o settings.o planner.o magazine.o \
             nuts_bolts.o limits.o print.o probe.o report.o system.o \
             counters.o gqueue.o progman.o adc.o spi.o signals.o systick.o \
             motor_driver.o

# FUSES      = -U hfuse:w:0xd9:m -U lfuse:w:0x24:m
FUSES      = -U hfuse:w:0xd8:m -U lfuse:w:0xff:m
# update that line with this when programmer is back up:
# FUSES      = -U hfuse:w:0xd7:m -U lfuse:w:0xff:m

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude  $(PROGRAMMER) -p $(DEVICE) -B 10 -F -D
CFLAGS = -Wall -Werror -Wextra -Os -fstack-usage --std=c11 \
	-DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -DGRBL_VERSION=$(VERSION)
COMPILE = avr-gcc $(CFLAGS) -I. -ffunction-sections

ifneq ($(SPI),)
  CFLAGS += -DSPI_STEPPER_DRIVER
endif

ifneq ($(USE_CAROUSEL_LOSS),)
  CFLAGS += -DUSE_CAROUSEL_LOSS
endif

# symbolic targets:
all debug:	grbl.hex

debug:	CFLAGS += -g

.c.o:
	$(COMPILE) -c $< -o $@
	@$(COMPILE) -MM  $< > $*.d

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:grbl.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# Using a USB bootloader for ARM microcontrollers
load: all
	bootloadHID grbl.hex

clean:
	rm -f grbl.hex main.elf $(OBJECTS) $(OBJECTS:.o=.d) grbl.stack *.su grbl.cflow grbl_callgraph.pdf

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS) -lm -Wl,--gc-sections

grbl.stack: main.elf
	cat $(OBJECTS:.o=.su) > $@

grbl.cflow: main.elf
	cflow $(OBJECTS:.o=.c) -o $@

grbl_callgraph.pdf: grbl.cflow grbl.stack
	./script/stackusage.py -s grbl.stack -f grbl.cflow -c grbl_callgraph.pdf

stack_analysis: grbl.stack grbl.cflow
	./script/stackusage.py -s grbl.stack -f grbl.cflow -a

grbl.hex: main.elf
	rm -f grbl.hex
	avr-objcopy -j .text -j .data -O ihex main.elf grbl.hex
	avr-size --format=berkeley main.elf

# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c

# include generated header dependencies
-include $(OBJECTS:.o=.d)
