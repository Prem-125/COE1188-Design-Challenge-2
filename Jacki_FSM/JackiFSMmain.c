//Team Gamma - Design Project 2

#include <stdint.h>
#include "msp.h"
#include "../inc/SysTickInts.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
#include "../inc/TimerA1.h"
#include "../inc/FlashProgram.h"
#include "../inc/IRDistance.h"
#include "../inc/LPF.h"
#include "../inc/ADC14.h"
#include "../inc/Tachometer.h"
#include "../inc/TA2InputCapture.h"

#define RAM_SIZE (256)
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))

volatile uint32_t nr,nc,nl; // filtered ADC samples
uint32_t Right,Center,Left; // distance in mm


uint8_t ramTrk=0, LineReading,index, Readings[10];
uint16_t ramVals[RAM_SIZE];
uint32_t romTrk= 0x20000, Time, MainCount;


struct State {
  uint8_t color;
  void (*func)();
  const struct State *next[4];
};
typedef const struct State State_t;

//State assignments
#define Travel          &fsm[0]
#define FoundTreasure   &fsm[1]
#define TurnRight       &fsm[2]
#define TurnLeft        &fsm[3]

//LED Color Encodings
#define RED       0x01
#define GREEN     0x02
#define YELLOW    0x03
#define BLUE      0x04
#define CLEAR     0x00

State_t *Spt;  // pointer to the current state

void Turn_Left(){
  SysTick->CTRL = 0;
  Motor_Left(7000, 7000);
  Clock_Delay1us(20000);
  Motor_Forward(6000,6000);
  Clock_Delay1us(5000);
  SysTick->CTRL = 0x00000007;
}
void Turn_Right(){
  SysTick->CTRL = 0;
  Motor_Right(7000,7000);
  Clock_Delay1us(20000);
  Motor_Forward(7000,7000);
  Clock_Delay1us(5000);
  SysTick->CTRL = 0x00000007;
}

//LED Init and Output functions
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

void Travelling()
{

}

State_t fsm[4] = {
   {BLUE, &Travelling,{Travel,Travel,Travel, Travel}}, // Travel
   {BLUE, &Travelling,{Travel,Travel,Travel, Travel}}, // Travel
   {BLUE, &Travelling,{Travel,Travel,Travel, Travel}}, // Travel
   {BLUE, &Travelling,{Travel,Travel,Travel, Travel}}, // TurnLeft
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

//SUBJECT TO CHANGE
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
    }

    Center = CenterConvert(nc);
    Left = LeftConvert(nl);
    Right = RightConvert(nr);

    Time++;
}


void Center_Handler(uint16_t time)
{
    nc = LPF_Calc2(time);
}

void Left_Handler(uint16_t time)
{
    nl = LPF_Calc3(time);
}

void Right_Handler(uint16_t time)
{
    nr = LPF_Calc(time);
}

void Trigger_Init()
{
    //Initialize P2.7 as the trigger (output)
    P2 -> SEL0 &= ~0x80;
    P2 -> SEL1 &= ~0x80;
    P2 -> DIR  |= 0x80;
    P2 -> OUT  &= ~0x80;
}

void Trigger_Handler()
{
    P2 -> OUT &= ~0x80;
    Clock_Delay1us(2);
    P2 -> OUT |= 0x80;
    Clock_Delay1us(15);
    P2 -> OUT &= ~0x80;

    //Reset timer
    TimerA2Capture_Init(&Center_Handler, &Left_Handler, &Right_Handler);
}

void main(void){

    DisableInterrupts();
    Clock_Init48MHz();

    //Low pass filter initializers
    LPF_Init(0, 64);  // P9.0/channel 17, 64-wide FIR filter
    LPF_Init2(0, 64); // P6.1/channel 14, 64-wide FIR filter
    LPF_Init3(0, 64);

    //Setting IR sampling to the timer
    TimerA1_Init(&Trigger_Handler, 125);

    //Timer for Wall Sensors (5.6 = 1, 5.7 = 2, 6.6 = 3)
    TimerA2Capture_Init(&Center_Handler, &Left_Handler, &Right_Handler);

    //Other
    Debug_Init();
    Debug_FlashInit();
    Time = MainCount = LineReading = index = 0;
    SysTick_Init(48000, 0);
    Reflectance_Init();
    Port2_Init();
    Motor_Init();
    Bump_Init();
    Tachometer_Init();
    EnableInterrupts();

    Spt = Travel;
    while(1){
        WaitForInterrupt();
        MainCount++;
    }
}

