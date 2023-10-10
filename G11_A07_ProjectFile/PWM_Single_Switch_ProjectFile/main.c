#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

//Using M1PWM6, on Pin PF2; controlled by M1PWM6, controlled by Module 1 PWM Generator 3.

void PortFConfig(void);
void PWMConfig(void);
void one_switch(void);
void GPIOF_Handler(void);
void SysTick_Handler(void);

#define frequency 100000
#define time_period (16000000/frequency)
#define del_duty 5
volatile int duty;

#define Sw_Bits 0x11

//Systick Stuff
#define STCTRL *((volatile long *) 0xE000E010)      //Control and Status Register
#define STRELOAD *((volatile long *) 0xE000E014)   //SysTick Reload Value Register
#define STCURRENT *((volatile long *) 0xE000E018)  //SysTick Current Value Register

//Definitions to configure systick CSR(Control and Status Register)
#define ENABLE (1<<0)       //bit 0 of CSR enables systick counter
#define DISABLE (1<<0)       //bit 0 of CSR disables systick counter
#define INT_EN (1<<1)       //bit 1 of CSR to generate interrupt to the NVIC when SysTick counts to 0
#define Clk_SRC (1<<2)      //bit 2 of CSR to select system clock
#define COUNT_FLAG (1<<16)  //bit 16 of CSR; The SysTick timer has counted to 0 since the last time this bit was read.

void main(void)

{
    PortFConfig();
    PWMConfig();
    duty=50;
    PWM1_3_CMPA_R = (time_period * duty) / 100;     //Provide initial duty to pwm
    while(1){}
}

void PortFConfig(void)
{
    SYSCTL_RCGCGPIO_R |= (1<<5);        //Enable and provide a clock to GPIO Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B;     //Unlock PortF register
    GPIO_PORTF_CR_R = 0x1F;             //Enable Commit function

    GPIO_PORTF_PUR_R = 0x11;            //Pull-up for user switches
    GPIO_PORTF_DEN_R = 0x1F;            //Enable all pins on port F
    GPIO_PORTF_DIR_R = 0x0E;            //Define PortF LEDs as output and switches as input

    //PortF Interrupt Configurations: User Sw should trigger hardware interrupt
    GPIO_PORTF_IS_R &= ~Sw_Bits;        //Edge trigger detected
    GPIO_PORTF_IBE_R &= ~Sw_Bits;     //Trigger interrupt according to GPIOIEV
    GPIO_PORTF_IEV_R &= ~Sw_Bits;     //Trigger interrupt on falling edge
    GPIO_PORTF_IM_R &= ~Sw_Bits;        //Mask interrupt bits
    GPIO_PORTF_ICR_R |= Sw_Bits;        //clear any prior interrupts
    GPIO_PORTF_IM_R |= Sw_Bits;         //enable interrupts for bits corresponding to Mask_Bits

    //NVIC Configuration
    //PortF interrupts correspond to interrupt 30 (EN0 and PRI7 registers)
    NVIC_EN0_R |= (1<<30);              //Interrupts enabled for port F
    NVIC_PRI7_R &= 0xFF3FFFFF;          //Interrupt Priority 1 to Port F
}

void PWMConfig(void)
{
    SYSCTL_RCGCPWM_R |= (1<<1);     //Enable and provide a clock to PWM module 1
    GPIO_PORTF_AFSEL_R |= (1<<2);   //Enable Alternate function for PF2
    GPIO_PORTF_PCTL_R |= 0x500;     //Selecting PWM function for PF2 (Page 1351)

    PWM1_3_CTL_R |= 0x00;
    PWM1_3_GENA_R = 0x8C;          //Configure PWM Gen A such that PWMA is High when counter=LOAD and low when counter=CMPA when counting down
    PWM1_3_LOAD_R = time_period;
    PWM1_3_CMPA_R = (duty/100)*time_period - 1;
    PWM1_3_CTL_R |= 0x01;
    PWM1_ENABLE_R |= 0x040;
}

void GPIOF_Handler(void)
{
    one_switch();

    int j;
    for(j = 0; j <1600*1000/4; j++){}           //Debounce Delay of 0.25seconds
}

void one_switch(void)
{
    GPIO_PORTF_IM_R &= ~Sw_Bits;

    STCURRENT = 0x00;                       //Reinitialise Systick Counter to Zero
    STRELOAD = (16*1000000);                //1 Second countdown
    STCTRL |= (ENABLE | INT_EN | Clk_SRC);  //Enable Systick, Enable Interrupt Generation, Enable system clock (80MHz) as source
}

void SysTick_Handler(void)
{
    STCTRL &= DISABLE;
    if((GPIO_PORTF_DATA_R) & (1<<4))        //Long Press
        {
            if(duty > 5)
            {
                duty = duty - del_duty;         //Decrease duty
            }
        }
    else                                    //Short Press
        {
            if(duty < 95)
            {
                duty = duty + del_duty;         //Increase duty
            }
        }
    PWM1_3_CMPA_R = (time_period * duty) / 100;
    GPIO_PORTF_ICR_R = Sw_Bits;
    GPIO_PORTF_IM_R |= Sw_Bits;
}
