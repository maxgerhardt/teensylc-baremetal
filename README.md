
Bare-Metal Teensy LC toolchain
--------------------------------

Type "make" to compile and link the sample main.cpp program.
"make burn" will load the resulting main.hex file into your
Teensy LC using the cli teensy loader.  The program
turns on your Teensy LC LED.

The makefile will compile and link all source files (.c, .S, and .cpp)
found in the working directory together into one hex file.  This hex file
is named after the TARGET variable given in the makefile.

A sample main.c version of the main.cpp file in the repo
is given below.


     #include "kinetis.h"

     #define LED  (1U << 5)

     int main(void)
     {
         SIM_SCGC5 = SIM_SCGC5_PORTC;
         PORTC_PCR5 = PORT_PCR_MUX(1U);
         GPIOC_PDDR |= LED;
         GPIOC_PSOR = LED;
         return 0;
     }


Remember, you must ensure any port used has its clock line enabled. 
For example, with main.cpp the clock line for port c must be enabled 
in order to light the LED.  The toolchain code does not enable
any clock lines.

The ARM CMSIS library files are also provided.  See main.cpp 
to enable an example of their use.

The following files were copied from the Teensyduino plugin.

     kinetis.h
     makefile (modified)
     mk20dx128.c (modified)
     mkl26z64.ld (modified)

The following files were copied over from the gcc4mbed repo:

     core_cm0plus.h
     core_cmFunc.h
     core_cmInstr.h
     MKL26Z4.h
     system_MKL26Z4.h

- George

Copyright (c) 2016 roseengineering 

