/**
 * @file Timer_A2_Capture.c
 * @brief Source code for the Timer_A2_Capture driver.
 *
 * This file contains the function definitions for configuring Timer A2 in Capture mode.
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/Timer_A2_Capture.h"

// User-defined function
void (*Timer_A2_Capture_Task)(uint16_t time);

// Initialize Timer A2 in Capture mode to request interrupts on the rising edge
// of the external signal. The ISR will acknowledge the interrupt and calls a user-defined function
void Timer_A2_Capture_Init(void(*task)(uint16_t time))
{
    Timer_A2_Capture_Task = task;
     P5->SEL0 |= 0x40;
     P5->SEL1 &= ~0x40;
     P5->DIR &= ~0x40;
     TIMER_A2->CTL &= ~0x0030;
     TIMER_A2->CTL = 0x0200;
     TIMER_A2->CCTL[1] = 0x4910;
     TIMER_A2->EX0 &= ~0x0000;
    // Set Interrupt Priority Level to 3
    NVIC->IP[3] = (NVIC->IP[3] & 0xFFFF0FFF) | 0x00006000;
    // Enable Interrupt 13 in NVIC
     NVIC->ISER[0] = 0x00002000;
    // Reset and start Timer A2 in Continuous mode
     TIMER_A2->CTL |= 0x0024;
}


void TA2_N_IRQHandler(void)
{
    // Acknowledge Capture/Compare interrupt and clear it
    TIMER_A2->CCTL[1] &= ~0x0001;

    // Execute the user-defined task
    (*Timer_A2_Capture_Task)(TIMER_A2->CCR[1]);
}
