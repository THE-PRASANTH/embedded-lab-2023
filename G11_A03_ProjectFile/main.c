#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define STCTRL *((volatile long *) 0xE000E010)    // control and status
#define STRELOAD *((volatile long *) 0xE000E014)    // reload value
#define STCURRENT *((volatile long *) 0xE000E018)    // current value

#define COUNT_FLAG  (1 << 16)   // bit 16 of CSR automatically set to 1
                                //   when timer expires
#define ENABLE      (1 << 0)    // bit 0 of CSR to enable the timer
#define CLKINT      (1 << 2)    // bit 2 of CSR to specify CPU clock

#define CLOCK_MHZ 16

void Delay(int us)
{
    STCURRENT = 0;
    STRELOAD = us*16;                       // reload value for 'us' microseconds
    STCTRL |= (CLKINT | ENABLE);        // set internal clock, enable the timer

    while ((STCTRL & COUNT_FLAG) == 0)  // wait until flag is set
    {
        ;   // do nothing
    }
    STCTRL = 0;                // stop the timer

    return;
}

int main(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;       /* enable clock to GPIOF */
    GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    GPIO_PORTF_CR_R = 0x1F;             /* make PORTF0 configurable */
    GPIO_PORTF_DEN_R = 0x1F;            /* set PORTF pins 4-3-2-1-0 as digital pins */
    GPIO_PORTF_DIR_R = 0x0E;            /* set PORTF3+PORTF2+PORTF1 pin as output (LED) pin */

    while(1){
     GPIO_PORTF_DATA_R = 0X0E;          /* White */
     Delay(200);			//Original Delay
     //Delay(200-9);			//Modified Delay     
     GPIO_PORTF_DATA_R = 0X00;          /* Dark */
     Delay(800);
    }
    return 0;
}
