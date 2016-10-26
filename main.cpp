
#if 1

#include "kinetis.h"

#define LED  (1U << 5)

extern "C" int main(void)
{
    SIM_SCGC5 = SIM_SCGC5_PORTC;
    PORTC_PCR5 = PORT_PCR_MUX(1U);
    GPIOC_PDDR |= LED;
    GPIOC_PSOR = LED;
}

#else

#include "MKL26Z4.h"

#define LED  (1U << 5)

extern "C" int main(void)
{
    SIM->SCGC5 = SIM_SCGC5_PORTC_MASK;
    PORTC->PCR[5] = PORT_PCR_MUX(1U);
    PTC->PDDR |= LED;
    PTC->PSOR = LED;
}

#endif


