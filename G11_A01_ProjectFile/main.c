#include <stdint.h>
#include "tm4c123gh6pm.h"


int main(void)
{
    SYSCTL_RCGCGPIO_R |= 0x02;  // enable clock to GPIOB
    GPIO_PORTB_DIR_R |= 0x02;   // set PB1 as output
    GPIO_PORTB_DEN_R |= 0x02;   // enable digital function for PB1
    int i,x;
    x=SYSCTL_RCC_R;
    while(1)
    {
       GPIO_PORTB_DATA_R = 0x02;   // toggle PB1
       // delay for 1 millisecond
       for(i = 0; i <800; i++){}
       // delay for 1 millisecond
       GPIO_PORTB_DATA_R= 0x00;     // toggle PB1
       for(i = 0; i <800; i++){}
    }
}
