// MotorSimple.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.
// Solution to Lab 12, uses Systick software delay to create PWM
// Daniel Valvano
// July 11, 2019

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



//**************RSLK1.1***************************
// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "../inc/Bump.h"
#include "../inc/SysTick.h"

// *******Lab 12 solution*******

void Motor_InitSimple(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs
    // write this as part of Lab 12

  P5->SEL0 &= ~0x30;
  P5->SEL1 &= ~0x30;  // configure as GPIO
  P5->DIR |= 0x30;    // make pins out
  P5->OUT &= ~0x30;
  // initialize P2.6 and P2.7 and make them outputs
  P2->SEL0 &= ~0xC0;
  P2->SEL1 &= ~0xC0;  // configure as GPIO
  P2->DIR |= 0xC0;    // make pins out
  P2->OUT &= ~0xC0;   // P2.6, P2.7 low
  // initialize P3.6 and P3.7 and make them outputs
  P3->SEL0 &= ~0xC0;
  P3->SEL1 &= ~0xC0;  // configure as GPIO
  P3->DIR |= 0xC0;    // make pins out
  P3->OUT &= ~0xC0;   // low current sleep mode
  SysTick_Init();
  Bump_Init();
}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
    // write this as part of Lab 12

  P5->OUT &= ~0x30;
  P2->OUT &= ~0xC0;   // off
  P3->OUT &= ~0xC0;   // low current sleep mode
}
void Motor_ForwardSimple(uint16_t duty, uint32_t time){int i;
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
// write this as part of Lab 12

  P5->OUT &= ~0x30;   // set direction of both motors
  P3->OUT |= 0xC0;    // activate both motors

  for(i=0;i<time;i++){
    if(Bump_Read()) break; // stop on bump switch
    P2->OUT |= 0xC0;       // both motors on
    SysTick_Wait(48*duty);
    P2->OUT &= ~0xC0;      // both motors off
    SysTick_Wait(48*(10000-duty));
  }
  Motor_StopSimple();
}
void Motor_BackwardSimple(uint16_t duty, uint32_t time){int i;
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Runs even if any bumper switch is active
// Returns after time*10ms
// write this as part of Lab 12

  P5->OUT |= 0x30;    // set back direction of both motors
  P3->OUT |= 0xC0;    // activate both motors

  for(i=0;i<time;i++){
    P2->OUT |= 0xC0;       // both motors on
    SysTick_Wait(48*duty);
    P2->OUT &= ~0xC0;      // both motors off
    SysTick_Wait(48*(10000-duty));
  }
  Motor_StopSimple();
}
void Motor_LeftSimple(uint16_t duty, uint32_t time){ int i;
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
// write this as part of Lab 12

  P5->OUT &= ~0x10;   // set direction of left motor
  P3->OUT &= ~0x40;   // deactivate right motor
  P3->OUT |= 0x80;    // activate left motor

  for(i=0;i<time;i++){
    if(Bump_Read()) break; // stop on bump switch
    P2->OUT |= 0x80;   // left on
    SysTick_Wait(48*duty);
    P2->OUT &= ~0x80;  // left off
    SysTick_Wait(48*(10000-duty));
  }
  Motor_StopSimple();
}
void Motor_RightSimple(uint16_t duty, uint32_t time){int i;
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
// write this as part of Lab 12

  P5->OUT &= ~0x20;   // set direction of right motor
  P3->OUT &= ~0x80;   // deactivate left motor
  P3->OUT |= 0x40;    // activate right motor

  for(i=0;i<time;i++){
    if(Bump_Read()) break; // stop on bump switch
    P2->OUT |= 0x40;   // right on
    SysTick_Wait(48*duty);
    P2->OUT &= ~0x40;  // right off
    SysTick_Wait(48*(10000-duty));
  }
  Motor_StopSimple();
}
