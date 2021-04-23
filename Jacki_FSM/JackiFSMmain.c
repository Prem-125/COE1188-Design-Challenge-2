//Team Gamma - Design Project 2

#include <stdint.h>
#include <stdbool.h>
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
#include "../inc/EUSCIA0.h"

#define RAM_SIZE (256)
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))

volatile uint32_t nr,nc,nl; // filtered ADC samples
uint32_t Right,Center,Left; // distance in mm


uint8_t ramTrk=0, LineReading,index, Readings[10];
uint16_t ramVals[RAM_SIZE];
uint32_t romTrk= 0x20000, Time, MainCount;

struct State {
  void (*func)();                 // time to delay in 1ms
  const struct State *next[5]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Straight        &fsm[0]
#define RightTurn       &fsm[1]
#define LeftTurn        &fsm[2]
#define CircleTurn      &fsm[3]
#define FoundTreasure   &fsm[4]

State_t *Spt;  // pointer to the current state

bool FWall,RWall,LWall;


void Turn_Left(){
  SysTick->CTRL = 0;
  Motor_Forward(7000,7000);
  Clock_Delay1us(5000);
  Motor_Left(7000, 7000);
  Clock_Delay1us(20000);
  Motor_Forward(7000,7000);
  Clock_Delay1us(5000);
  SysTick->CTRL = 0x00000007;
}
void Turn_Right(){
  SysTick->CTRL = 0;
  Motor_Forward(7000,7000);
  Clock_Delay1us(5000);
  Motor_Right(7000,7000);
  Clock_Delay1us(20000);
  Motor_Forward(7000,7000);
  Clock_Delay1us(5000);
  SysTick->CTRL = 0x00000007;
}

void Turn_180(){
  SysTick->CTRL = 0;
  Motor_Right(7000,7000);
  Clock_Delay1us(40000);
  SysTick->CTRL = 0x00000007;

}

//LED Init and Output functions
void Port2_Init(void){
  P2->SEL0 &= ~0x07;
  P2->SEL1 &= ~0x07;                        // configure P2.2-P2.0 as GPIO
  P2->DS |= 0x07;                          // make P2.2-P2.0 high drive strength
  P2->DIR |= 0x07;                         // make P2.2-P2.0 out
  P2->OUT |= 0x07;                         // all LEDs off
}

void Port2_Output(uint8_t data){        // write all of P2 outputs
  P2->OUT = data;
}

uint16_t accumulator = 0;
uint16_t oldError = 0;
#define P 9
#define I 0
#define D 0
#define BaseSpeed 7000
uint16_t diff= 0;

void Travelling()
{
    uint16_t error = Left - Right;
    accumulator += error;
    uint16_t deltaError = error - oldError;

    diff = 0;
    diff += error * P;
    diff += accumulator * I;
    diff += deltaError * D;
    //Motor_Forward(6000,6000);
    uint16_t lspeed = BaseSpeed - diff;
    uint16_t rspeed = BaseSpeed + diff;
    Motor_Forward(lspeed,rspeed);

    oldError = error;
}

void Found_Treasure(){

}

State_t fsm[5] = {
   {&Travelling,{Straight, RightTurn, LeftTurn, CircleTurn,FoundTreasure}}, // Straight
   {&Turn_Right,{Straight, RightTurn, LeftTurn, CircleTurn,FoundTreasure}}, // RightTurn
   {&Turn_Left,{Straight, RightTurn, LeftTurn, CircleTurn,FoundTreasure}}, // LeftTurn
   {&Turn_180,{Straight, RightTurn, LeftTurn, CircleTurn,FoundTreasure}}, //CircleTurn
   {&Found_Treasure,{Straight, RightTurn, LeftTurn, CircleTurn,FoundTreasure}} //CircleTurn
};

uint8_t StateSel(void){
    /*
    if(LineReading!= 0x3F){ // Treasure Found
        return 4;
    }
    */
    if(FWall && RWall && LWall){ // Dead End
        return 3;
    }
    if (!FWall && RWall && LWall){ // Corridor
        return 0;
    }

    if (FWall && RWall &!LWall){// 90 deg left
        return 2;
    }

    if (FWall && !RWall &LWall){// 90 deg right
        return 1;
    }
    if (RWall) return 1;
    return 0;

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

#define DETECTFW 100
#define DETECTL 150
#define DETECTR 150

uint16_t LTIME = 0;
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

    FWall = (Center < DETECTFW && Center > 30 ? true : false);
    RWall = (Right < DETECTR && Right > 0 ? true : false);
    LWall = (Left < DETECTL && Left > 0 ? true : false);

    index = StateSel();
    Spt=Spt->next[index];
    (*Spt->func)();

    //sel function
    if(Time % 1000 == 0){
        EUSCIA0_OutString("\nDebug MSG\n");
        EUSCIA0_OutString("\nLeft\n");

    EUSCIA0_OutUDec(Left); EUSCIA0_OutChar(LF);
    EUSCIA0_OutUDec(nl); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString("\nDiff\n");

    EUSCIA0_OutUDec(diff); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString("\nRight\n");

    EUSCIA0_OutUDec(Right); EUSCIA0_OutChar(LF);
    EUSCIA0_OutUDec(nr); EUSCIA0_OutChar(LF);
   // EUSCIA0_OutUDec(Right); EUSCIA0_OutChar(LF);
    }
    Time++;
}


void Center_Handler(uint16_t timeIn)
{
    nc = LPF_Calc2(timeIn);
}

void Left_Handler(uint16_t timeIn)
{
    nl = LPF_Calc3(timeIn);
    LTIME = timeIn;
}

void Right_Handler(uint16_t timeIn)
{
    nr = LPF_Calc(timeIn);
}

void Trigger_Init()
{
    //Initialize P3.5 as the trigger (output)
    P3 -> SEL0 &= ~0x20;
    P3 -> SEL1 &= ~0x20;
    P3 -> DIR  |= 0x20;
    P3 -> OUT  &= ~0x20;
}

void Trigger_Handler()
{
    DisableInterrupts();

    P3 -> OUT &= ~0x20;
    Clock_Delay1us(2);
    P3 -> OUT |= 0x20;
    Clock_Delay1us(1000);
    P3 -> OUT &= ~0x20;
    //Reset timer
    TimerA2Capture_Init(&Center_Handler, &Left_Handler, &Right_Handler);
    EnableInterrupts();

}

void main(void){

    DisableInterrupts();
    Clock_Init48MHz();

    //Low pass filter initializers
    LPF_Init(100, 8);
    LPF_Init2(100, 8);
    LPF_Init3(100, 8);

    //Setting IR sampling to the timer
    Trigger_Init();
    TimerA1_Init(&Trigger_Handler, 10000);

    //Timer for Wall Sensors (5.6 = 1, 5.7 = 2, 6.6 = 3)
    TimerA2Capture_Init(&Center_Handler, &Left_Handler, &Right_Handler);

    //Enable UART Comms
    EUSCIA0_Init();

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
    //EUSCIA0_OutString("\nStarting\n");

    Spt = Straight;
    while(1){
        WaitForInterrupt();
        MainCount++;
    }
}

