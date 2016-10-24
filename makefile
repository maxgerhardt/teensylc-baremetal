# The name of your project (used to name the compiled .hex file)
TARGET = main

# set your MCU type here, or make command line `make MCU=MK20DX256`
MCU=MKL26Z64

# make it lower case
LOWER_MCU := $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$(MCU)))))))))))))))))))))))))))
MCU_LD = $(LOWER_MCU).ld

# configurable options
OPTIONS = -DF_CPU=48000000 -D__$(MCU)__ 

# Other Makefiles and project templates for Teensy 3.x:
#
# https://github.com/apmorton/teensy-template
# https://github.com/xxxajk/Arduino_Makefile_master
# https://github.com/JonHylands/uCee

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -nostdlib -Wall -Os -mthumb -g -MMD $(OPTIONS) -I. -mcpu=cortex-m0plus -nostdlib -fsingle-precision-constant 

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections 

ASFLAGS = -x assembler-with-cpp 

# compiler options for C only
CFLAGS =

# linker options
LDFLAGS = -Os -Wl,--gc-sections,--no-wchar-size-warning --specs=nano.specs -mcpu=cortex-m0plus -mthumb -T$(MCU_LD)

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
	$(OBJDUMP) -S $< > $(TARGET).lst

# compiler generated dependency info
-include $(OBJS:.o=.d)

burn: $(TARGET).hex
	teensy_loader_cli -mmcu=mkl26z64 -w -v $<
clean:
	rm -f *.o *.d $(TARGET).elf $(TARGET).hex $(TARGET).lst
zip:
	zip -FS -r $(PWD) . -x @.gitignore -x '.git/*'


