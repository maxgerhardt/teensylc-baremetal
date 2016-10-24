
#include "kinetis.h"

#define LED  (1U << 5)

extern "C" int main(void)
{
    SIM_SCGC5 = SIM_SCGC5_PORTC;
    PORTC_PCR5 = PORT_PCR_MUX(1U);
    GPIOC_PDDR |= LED;
    GPIOC_PSOR = LED;
}

