// GPIO Input Experiment

/**
 * main.c
 */

/*
1.
Press Button, LED red, or else off

2.
Press button, LED: R->B->G (cycle), Implement debounce
*/
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define Red_Led 1
#define Blue_Led 2
#define Green_Led 3

int main(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;       /* enable clock to GPIOF */
    GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    //GPIO_PORTF_CR_R = 0x1F;             /* make PORTF0 configurable */
    GPIO_PORTF_DEN_R = 0x1F;            /* set PORTF pins 4-3-2-1-0 as digital pins */
    GPIO_PORTF_PUR_R = 0x11;            /* enable pull up for pin 4 and 0 */
    GPIO_PORTF_DIR_R = 0x0E;            /* set PORTF3+PORTF2+PORTF1 pin as output (LED) pin and UsrSw1-UsrSw2 as input */


    while(1){

        //if(GPIO_PORTF_DATA_R & 0x10)                //Check if only UsrSw1 is set
        if(!(GPIO_PORTF_DATA_R & 0x10))                //Check if only UsrSw1 is set
            {
            GPIO_PORTF_DATA_R |= (1 << Red_Led);          //Only RED LED on
            }
        else
            {
            GPIO_PORTF_DATA_R &= (0 << Red_Led);          //Only RED LED Off
            }

    }
    return 0;
}
