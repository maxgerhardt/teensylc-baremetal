APPNAME = $(shell basename $(PWD))

# The name of your project (used to name the compiled .hex file)
TARGET = main

# set your MCU type here, or make command line `make MCU=MK20DX256`
MCU=MKL26Z64
MCU_LD = $(MCU).ld

# configurable options
OPTIONS = -DF_CPU=48000000 -D__$(MCU)__ 

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -mthumb -g -MMD $(OPTIONS) -I. -mcpu=cortex-m0plus -nostdlib -fsingle-precision-constant

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections 

ASFLAGS = -x assembler-with-cpp 

# compiler options for C only
CFLAGS =

# linker options
LDFLAGS = -nostartfiles -Wl,--gc-sections,--no-wchar-size-warning --specs=nano.specs -mcpu=cortex-m0plus -mthumb -T$(MCU_LD)

# additional libraries to link
LIBS = -lm

# names for the compiler programs
CC = $(COMPILERPATH)/arm-none-eabi-gcc
CXX = $(COMPILERPATH)/arm-none-eabi-g++
OBJCOPY = $(COMPILERPATH)/arm-none-eabi-objcopy
SIZE = $(COMPILERPATH)/arm-none-eabi-size

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

# automatically create lists of the sources and objects
C_FILES := $(wildcard *.c)
S_FILES := $(wildcard *.S)
CPP_FILES := $(wildcard *.cpp)

OBJS := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(S_FILES:.S=.o)

# the actual makefile rules (all .o files built by GNU make's default implicit rules)

all: $(TARGET).hex

$(TARGET).elf: $(OBJS) $(MCU_LD)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	readelf -a $< > $(TARGET).lst
	$(OBJDUMP) -x -S $< >> $(TARGET).lst
	$(OBJDUMP) -x -j .data $< >> $(TARGET).lst
	$(OBJDUMP) -x -j .bss $< >> $(TARGET).lst
	readelf -x .data $< >> $(TARGET).lst

# compiler generated dependency info
-include $(OBJS:.o=.d)

burn: $(TARGET).hex
	teensy_loader_cli -mmcu=$(MCU) -w -v $<
clean:
	rm -f *.o *.d *.elf *.hex *.lst
zip:
	(cd ..; \
        zip -FS -r $(APPNAME) $(APPNAME) \
        -x @'$(APPNAME)/.gitignore' -x '$(APPNAME)/.git/*')

