/*
 * This code uses interrupts to blink an LED for one second whenever a user switch is pressed
 * It uses 2 interrupt services:
 * PortFHandler; for handling hardware interrupt when switch is pressed
 * SysTickHandler; for when systick timer runs out
 */

#include <stdint.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>

#define STCTRL *((volatile long *) 0xE000E010)      //Control and Status Register
#define STRELOAD *((volatile long *) 0xE000E014)   //SysTick Reload Value Register
#define STCURRENT *((volatile long *) 0xE000E018)  //SysTick Current Value Register

//Definitions to configure systick CSR(Control and Status Register)
#define ENABLE (1<<0)       //bit 0 of CSR enables systick counter
#define INT_EN (1<<1)       //bit 1 of CSR to generate interrupt to the NVIC when SysTick counts to 0
#define Clk_SRC (1<<2)      //bit 2 of CSR to select system clock
#define COUNT_FLAG (1<<16)  //bit 16 of CSR; The SysTick timer has counted to 0 since the last time this bit was read.

#define Mask_Bits 0x11

//Configure PortF; Enable clock to PortF; LEDs as digital output, Switches as digital input
void PortFConfig(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;   //enable clock to GPIOF
    GPIO_PORTF_LOCK_R = 0x4C4F434B; //Unlock PortF register
    GPIO_PORTF_CR_R = 0x1F;         //Enable Commit function

    GPIO_PORTF_DEN_R = 0x1F;        //Enable all pins on port F
    GPIO_PORTF_DIR_R = 0x0E;        //Set LEDs as outputs and Switches as inputs
    GPIO_PORTF_PUR_R = 0x11;        //Pull-up for user switches
}


void IntPortFHandler(void)
{
    GPIO_PORTF_ICR_R = Mask_Bits;           //Clear any previous interrupts on port F
    GPIO_PORTF_IM_R &= ~Mask_Bits;
    STCURRENT=0x00;                         //Reinitialise Systick Counter to Zero

    STRELOAD = 16*1000000;
    STCTRL |= (ENABLE | INT_EN | Clk_SRC);    //Enable Systick, Enable Interrupt Generation, Enable system clock (80MHz) as source
    GPIO_PORTF_DATA_R = 0x0E;
}

void SysTickHandler(void)
{
    GPIO_PORTF_DATA_R = 0x11;
    //mask, clear and unmask gpio interrupt
    GPIO_PORTF_IM_R &= ~Mask_Bits;
    GPIO_PORTF_ICR_R = Mask_Bits;
    GPIO_PORTF_IM_R |= Mask_Bits;
}


int main(void)
{
    PortFConfig();

    //PortF Interrupt Configurations: User Sw should trigger hardware interrupt
    GPIO_PORTF_IS_R &= ~Mask_Bits;      //Edge trigger detected
    GPIO_PORTF_IBE_R &= ~Mask_Bits;     //Trigger interrupt according to GPIOIEV
    GPIO_PORTF_IEV_R &= ~Mask_Bits;     //Trigger interrupt on falling edge
    GPIO_PORTF_IM_R &= ~Mask_Bits;      //Mask interrupt bits
    GPIO_PORTF_ICR_R |= Mask_Bits;       //clear any prior interrupts
    GPIO_PORTF_IM_R |= Mask_Bits;       //enable interrupts for bits corresponding to Mask_Bits

    //NVIC Configuration
    //PortF interrupts correspond to interrupt 30 (EN0 and PRI7 registers)
    NVIC_EN0_R |= (1<<30);              //Interrupts enabled for port F
    NVIC_PRI7_R &= 0xFF3FFFFF;          //Interrupt Priority 1 to Port F

    STCURRENT=0x00;                         //Reinitialise Systick Counter to Zero

    while(1);

}

