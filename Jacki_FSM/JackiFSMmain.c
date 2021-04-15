

//*****************************************************************************
//
// Jacki FSM test main
// MSP432 with Jacki
// Daniel and Jonathan Valvano
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

#include <stdint.h>
#include "msp.h"
#include "../inc/SysTickInts.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
//#include "../inc/SysTick.h"
#include "../inc/TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
#include "../inc/TimerA1.h"
#include "../inc/FlashProgram.h"

#define RAM_SIZE (256)
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))


uint8_t ramTrk=0, LineReading,index, Readings[10];
uint16_t ramVals[RAM_SIZE];
uint32_t romTrk= 0x20000, Time, MainCount;


struct State {
  uint8_t color;
  uint16_t left;                // 2-bit output
  uint16_t right;
  void (*func)(uint16_t,uint16_t);                 // time to delay in 1ms
  const struct State *next[12]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Straight    &fsm[0]
#define SoftRight   &fsm[1]
#define MidRight    &fsm[2]
#define HardRight   &fsm[3]
#define TurnRight   &fsm[4]
#define SoftLeft    &fsm[5]
#define MidLeft     &fsm[6]
#define HardLeft    &fsm[7]
#define TurnLeft    &fsm[8]
#define Backup      &fsm[9]
#define Stop        &fsm[10]

#define FSpeed 10000
#define SVSpeed (FSpeed - 100)
#define MVSpeed (FSpeed - 500)
#define TSpeed (FSpeed - 4000)
#define HVSpeed (FSpeed - 2000)

#define RED       0x01
#define GREEN     0x02
#define YELLOW    0x03
#define BLUE      0x04
#define CLEAR     0x00

State_t *Spt;  // pointer to the current state

void Turn_Left(uint16_t leftDuty, uint16_t rightDuty){
  SysTick->CTRL = 0;
  Motor_Left(leftDuty,rightDuty);
  Clock_Delay1us(20000);
  Motor_Forward(6000,6000);
  Clock_Delay1us(5000);
  SysTick->CTRL = 0x00000007;
}
void Turn_Right(uint16_t leftDuty, uint16_t rightDuty){
  SysTick->CTRL = 0;
  Motor_Right(leftDuty,rightDuty);
  Clock_Delay1us(20000);
  Motor_Forward(7000,7000);
  Clock_Delay1us(5000);
  SysTick->CTRL = 0x00000007;
}
void BackUp(uint16_t leftDuty, uint16_t rightDuty){
      SysTick->CTRL = 0;
      Motor_Backward(7000,7000);
      Clock_Delay1us(7000);
      SysTick->CTRL = 0x00000007;
}
void Stop_Motors(uint16_t leftDuty, uint16_t rightDuty){
      Motor_Stop();
}

void Port2_Init(void){
  P2->SEL0 = 0x00;
  P2->SEL1 = 0x00;                        // configure P2.2-P2.0 as GPIO
  P2->DS = 0x07;                          // make P2.2-P2.0 high drive strength
  P2->DIR = 0x07;                         // make P2.2-P2.0 out
  P2->OUT = 0x00;                         // all LEDs off
}

void Port2_Output(uint8_t data){        // write all of P2 outputs
  P2->OUT = data;
}




State_t fsm[11] = {
   {CLEAR, FSpeed,FSpeed,&Motor_Forward,{Stop,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Straight
   {BLUE, FSpeed,SVSpeed,&Motor_Forward,{Stop,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Soft Right
   {YELLOW, FSpeed,MVSpeed,&Motor_Forward,{Stop,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Mid Right
   {GREEN, FSpeed,HVSpeed,&Motor_Forward,{Backup,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Hard Right
   {RED, TSpeed,TSpeed,&Turn_Right,{Backup,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Turn Right
   {BLUE, SVSpeed,FSpeed,&Motor_Forward,{Stop,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Soft Left
   {YELLOW, MVSpeed,FSpeed,&Motor_Forward,{Stop,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Mid Left
   {GREEN, HVSpeed,FSpeed,&Motor_Forward,{Backup,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Hard Left
   {RED, TSpeed,TSpeed,&Turn_Left,{Backup,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}}, // Turn Left
   {CLEAR, TSpeed,TSpeed,&BackUp,{Backup,TurnLeft,TurnLeft,TurnLeft,TurnLeft,TurnLeft,TurnRight,TurnRight,TurnRight,TurnRight,Stop, Backup}}, // Backup
   {CLEAR, 0,0,&Stop_Motors,{Stop,TurnLeft,HardLeft,MidLeft,SoftLeft,Straight,SoftRight,MidRight,HardRight,TurnRight,Stop, Backup}} // Stop
};

uint8_t lineToState(uint8_t prev){
    if ( LineReading == 0x0){
        return 0;
    }
    else if (LineReading == 0x18 || LineReading == 0x1C || LineReading == 0x38 || LineReading == 0x3C)
    {
        return 5;
    }
    else if ( (LineReading == 0x30) || (LineReading == 0x10) || (LineReading == 0x20) )
    {
        return 4;
    }
    else if ( (LineReading == 0x60) || (LineReading == 0xE0) || (LineReading == 0x70) || (LineReading == 0x40)  )
    {
        return 3;
    }
    else if ( (LineReading == 0x80) )
    {
        return 2;
    }
    else if ( (LineReading == 0x0C) || (LineReading == 0x08) || (LineReading == 0x04) )
    {
        return 8;
    }
    else if ( (LineReading == 0x06) || (LineReading == 0x0E) || (LineReading == 0x07) || (LineReading == 0x03) || (LineReading == 0x02) )
    {
        return 6;
    }
    else if ( (LineReading == 0x01) )
    {
        return 8;
    }
    else if ( (LineReading == 0xF0) || (LineReading == 0xF8) || (LineReading == 0xFC) || (LineReading == 0xFE) )
    {
        return 1;
    }
    else if ( (LineReading == 0x0F) || (LineReading == 0x1F) || (LineReading == 0x3F)  || (LineReading == 0x7F) )
    {
        return 9;
    }
    else {
            return prev;
    }



}

void Debug_Init(void){
    int i;
    for (i = 0 ; i < RAM_SIZE; i ++){
        *(ramVals+i) = 0;
    }
    ramTrk=0;
}
void Debug_Dump(uint8_t x, uint8_t y){
    *(ramVals+ramTrk) = (((uint16_t)(x))<< 8) + y ;
    ramTrk = (ramTrk == RAM_SIZE - 1)? 0: ramTrk +1;
}
void Debug_FlashInit(void){
    uint32_t adr;
    for (adr = 0x20000; adr < 0x40000; adr+=0x1000)
        Flash_Erase(adr);
    romTrk= 0x20000;
}
void Debug_FlashRecord(uint16_t *pt){
    int i;
    if(romTrk < 0x40000){
    for(i =0; i < RAM_SIZE /32; i++ ){
    Flash_FastWrite((uint32_t *)(pt+32 * i),romTrk,16);
    romTrk += 64;
    }
    }
}

void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

void Task(void){
  //if(run == 15) TimerA1_Stop();
  //else run++;
}

void HandleCollision(uint8_t bump){
    if(bump != 0x3F)
    {
        //Stop the timer
        TimerA1_Stop();

        //Move away from the wall
        if(bump == 0x33)
        {
           //move backwards and spin 90 degrees
           Motor_Backward(3000, 3000);
           Clock_Delay1ms(1000);
           Motor_Right(3000,3000);
           Clock_Delay1ms(1000);
        }
        else if(bump > 7)
        {
           //move backwards and turn left
            Motor_Backward(3000, 3000);
            Clock_Delay1ms(1000);
            Motor_Right(3000,3000);
            Clock_Delay1ms(1000);
        }
        else
        {
           //move backwards and turn right
           Motor_Backward(3000, 3000);
           Clock_Delay1ms(1000);
           Motor_Left(3000,3000);
           Clock_Delay1ms(1000);
        }
    }

    TimerA1_Init(&Task,50000);
}

void PORT4_IRQHandler(void){
    P4 -> IFG &= 0x12; //clears pending flags
    HandleCollision(Bump_Read());
}


void SysTick_Handler(void){ // every 1ms
    if (Time % 5 == 0){
        Reflectance_Start();
    }
    else if (Time % 5 == 1){
        LineReading = Reflectance_End();
        index = lineToState(index);
        Spt=Spt->next[index];
        Debug_Dump(0, index);
        Port2_Output(Spt->color);
        (*Spt->func)(Spt->left,Spt->right);
    }
    Time++;
}

void main(void){
    Clock_Init48MHz();
    Debug_Init();
    Debug_FlashInit();
    Time = MainCount = LineReading = index = 0;
    SysTick_Init(48000, 0);
    Reflectance_Init();
    Port2_Init();
    Motor_Init();
    Bump_Init();
    TimerA1_Init(&Task, 50000);
    EnableInterrupts();
    Spt = Straight;
    while(1){
        WaitForInterrupt();
        MainCount++;
    }
}

