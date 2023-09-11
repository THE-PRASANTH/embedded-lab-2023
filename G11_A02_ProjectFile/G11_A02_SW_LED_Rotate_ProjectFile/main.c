// GPIO Input Experiment

/**
 * main.c
 */

/*
Press button, LED: R->B->G (cycle), Implement debounce
*/
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

int main(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;       /* enable clock to GPIOF */
    GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
    //GPIO_PORTF_CR_R = 0x1F;             /* make PORTF0 configurable */
    GPIO_PORTF_DEN_R = 0x1F;            /* set PORTF pins 4-3-2-1-0 as digital pins */
    GPIO_PORTF_PUR_R = 0x11;            /* enable pull up for pin 4 and 0 */
    GPIO_PORTF_DIR_R = 0x0E;            /* set PORTF3+PORTF2+PORTF1 pin as output (LED) pin and UsrSw1-UsrSw2 as input */


    int sw_old=0;
    int sw_new=0;
    int i=1;
    int j;

    while(1){
        for(j = 0; j <5000; j++){}               //Debounce Delay

        sw_new= !(GPIO_PORTF_DATA_R & 0x10);    //Take current switch value

        if(sw_new>sw_old)                       //Check for positive edge trigger
        {
            i++;
            if(i==4)
            {i=1;}
        }

        if(i==1)
        {GPIO_PORTF_DATA_R = 0X02;}             //Red
        if(i==2)
        {GPIO_PORTF_DATA_R = 0X04;}             //Blue
        if(i==3)
        {GPIO_PORTF_DATA_R = 0X08;}             //Green

        sw_old=sw_new;                          //Update old switch value

    }
    return 0;
}
