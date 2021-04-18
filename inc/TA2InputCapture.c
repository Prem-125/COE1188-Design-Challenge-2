// TA2InputCapture.c
// Runs on MSP432
// Use Timer A2 in capture mode to request interrupts on both
// edges of P5.6 (TA2CCP1) and call a user function.
// Daniel Valvano
// April 18, 2017
// Warning: CC3100 uses Timer A2

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// external signal connected to P5.6 (TA2CCP1) (trigger on both edges)

#include <stdint.h>
#include "../inc/CortexM.h"
#include "msp.h"

void ta2dummy(uint16_t t){};       // dummy function
void (*CaptureTaskk1)(uint16_t time) = ta2dummy;// user function
void (*CaptureTaskk2)(uint16_t time) = ta2dummy;// user function
void (*CaptureTaskk3)(uint16_t time) = ta2dummy;// user function

//------------TimerA2Capture_Init------------
// Initialize Timer A2 in edge time mode to request interrupts on
// both edges of P5.6 (TA2CCP1).  The interrupt service routine
// acknowledges the interrupt and calls a user function.
// Input: task is a pointer to a user function called when edge occurs
//             parameter is 16-bit up-counting timer value when edge occurred (units of 0.083 usec)
// Output: none
void TimerA2Capture_Init(void(*task1)(uint16_t time), void(*task2)(uint16_t time), void(*task3)(uint16_t time)){
  long sr;
  sr = StartCritical();
  CaptureTaskk1 = task1;             // user function
  CaptureTaskk2 = task2;
  CaptureTaskk3 = task3;
  //P5.6 = 1, P5.7 = 2, P6.6 = 3
  P5->SEL0 |= 0xC0;
  P5->SEL1 &= ~0xC0;               // configure P5.6 as TA2CCP1
  P5->DIR &= ~0xC0;                // make P5.6 in
  P6->SEL0 |= 0x40;
  P6->SEL1 &= ~0x40;               // configure P5.6 as TA2CCP1
  P6->DIR &= ~0x40;                // make P5.6 in
  TIMER_A2->CTL &= ~0x0030;        // halt Timer A2
  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=00,       input clock divider /1
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           interrupt disable
  // bit0=0,           clear interrupt pending
  TIMER_A2->CTL = 0x0200;
  // bits15-14=11,     capture on both edges
  // bits13-12=00,     capture/compare input on CCI2A
  // bit11=1,          synchronous capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=1,           capture mode
  // bits7-5=XXX,      output mode
  // bit4=1,           enable capture/compare interrupt
  // bit3=X,           read capture/compare input from here
  // bit2=X,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending
  TIMER_A2->CCTL[1] = 0x8910;
  TIMER_A2->CCTL[2] = 0x8910;
  TIMER_A2->CCTL[3] = 0x8910;
  TIMER_A2->EX0 &= ~0x0007;        // configure for input clock divider /1
  NVIC->IP[3] = (NVIC->IP[3]&0xFFFF00FF)|0x00004000; // priority 2
// interrupts enabled in the main program after all devices initialized
  NVIC->ISER[0] = 0x00002000;      // enable interrupt 13 in NVIC
  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=00,       input clock divider /1
  // bits5-4=10,       continuous count up mode
  // bit3=X,           reserved
  // bit2=1,           set this bit to clear
  // bit1=0,           interrupt disable (no interrupt on rollover)
  // bit0=0,           clear interrupt pending
  TIMER_A2->CTL |= 0x0024;         // reset and start Timer A2 in continuous up mode
  EndCritical(sr);
}

void TA2_N_IRQHandler(void){
    if(((TIMER_A2->CCTL[1])&0x0001) != 0){
        TIMER_A2->CCTL[1] &= ~0x0001;  // acknowledge capture/compare interrupt 2
        (*CaptureTaskk1)(TIMER_A2->CCR[1]);// execute user task
    }
    if(((TIMER_A2->CCTL[2])&0x0001) != 0){
            TIMER_A2->CCTL[2] &= ~0x0001;  // acknowledge capture/compare interrupt 2
            (*CaptureTaskk2)(TIMER_A2->CCR[2]);// execute user task
    }
    if(((TIMER_A3->CCTL[3])&0x0001) != 0){
            TIMER_A2->CCTL[3] &= ~0x0001;  // acknowledge capture/compare interrupt 2
            (*CaptureTaskk3)(TIMER_A2->CCR[3]);// execute user task
    }
}
