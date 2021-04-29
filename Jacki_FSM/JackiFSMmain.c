//Team Gamma - Design Project 2

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
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
#include "../inc/odometry.h"

//CONSTANTS NEEDED FOR ENTIRE PROGRAM
volatile uint32_t nr,nc,nl; // filtered ADC samples
uint32_t Right,Center,Left; // distance in mm
uint8_t LineReading;
uint32_t Time;

//COLOR DECLARATIONS TO INDICATE ROBOT STATE
#define RED       0x01
#define GREEN     0x02
#define YELLOW    0x03
#define BLUE      0x04
#define CLEAR     0x00

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


//FUNCTIONS TO HELP THE ROBOT NAVIGATE THROUGH THE MAZE

//Variables needed for PID control
uint16_t accumulator = 0;
uint16_t oldError = 0;
#define P 50
#define I 0
#define De -25
#define BaseSpeed 4000
#define DISTANCE_RIGHT_WALL 25
uint16_t diff= 0;

void Travelling()
{
    Port2_Output(GREEN);
    uint16_t error = Right - DISTANCE_RIGHT_WALL;
    accumulator += error;
    uint16_t deltaError = error - oldError;

    diff = 0;
    diff += error * P;
    diff += accumulator * I;
    diff += deltaError * De;
    uint16_t lspeed = BaseSpeed + diff;
    uint16_t rspeed = BaseSpeed - diff;
    Motor_Forward(lspeed,rspeed);

    oldError = error;
}

void Turn_Right(){

    //Go straight for a sec
    int32_t left;
    int32_t right;
    int32_t *leftTach = &left;
    int32_t *rightTach = &right;
    Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
    int32_t initLeft = *leftTach;
    int32_t initRight = *rightTach;
    int32_t deltaLeft = 0;
    int32_t deltaRight = 0;
    Port2_Output(RED);
    while(deltaLeft < 250 && deltaRight < 250)
    {
        Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
        deltaLeft = *leftTach -initLeft;
        deltaRight = *rightTach - initRight;
        Motor_Forward(3000,3000);
    }

    //Turn right
    Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
    initRight = *rightTach;
    deltaRight = 0;
    while(deltaRight > -170)
    {
        Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
        deltaRight = *rightTach -initRight;
        Motor_Right(3000,3000);
    }

    //Go straight for another sec
    Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
    initLeft = *leftTach;
    initRight = *rightTach;
    deltaLeft = 0;
    deltaRight = 0;
    while(deltaLeft < 250 && deltaRight < 250)
    {
        Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
        deltaLeft = *leftTach -initLeft;
        deltaRight = *rightTach - initRight;
        Motor_Forward(3000,3000);
    }
}

void Turn_Left(){

    //Go straight for a sec
    int32_t left;
    int32_t right;
    int32_t *leftTach = &left;
    int32_t *rightTach = &right;
    Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
    int32_t initLeft = *leftTach;
    int32_t initRight = *rightTach;
    int32_t deltaLeft = 0;
    int32_t deltaRight = 0;
    Port2_Output(YELLOW);
    /*
    while(deltaLeft < 250 && deltaRight < 250)
    {
        Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
        deltaLeft = *leftTach -initLeft;
        deltaRight = *rightTach - initRight;
        Motor_Forward(3000,3000);
    }
    */
    //Turn left
    Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
    initLeft = *leftTach;
    deltaLeft = 0;
    while(deltaLeft > -170)
    {
        Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
        deltaLeft = *leftTach -initLeft;
        Motor_Left(3000,3000);
    }

    //Go straight for another sec
    Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
    initLeft = *leftTach;
    initRight = *rightTach;
    deltaLeft = 0;
    deltaRight = 0;
    while(deltaLeft < 250 && deltaRight < 250)
    {
        Tachometer_Get(NULL, NULL,leftTach, NULL, NULL, rightTach);
        deltaLeft = *leftTach -initLeft;
        deltaRight = *rightTach - initRight;
        Motor_Forward(3000,3000);
    }
}

void Found_Treasure(){
    Motor_Stop();
    while(LineReading != 0x3F)
    {
        Port2_Output(RED);
        Clock_Delay1ms(500);
        Port2_Output(YELLOW);
        Clock_Delay1ms(500);
        Port2_Output(GREEN);
        Clock_Delay1ms(500);
        Port2_Output(BLUE);
        Clock_Delay1ms(500);
    }
}

void Task(void){
    //hi
}

int32_t ltime, rtime, ctime;
bool rRise = true, cRise = true, lRise = true;
void Center_Handler(int32_t timeIn1)
{
    if(cRise) ctime = timeIn1;
    else nc = LPF_Calc2(abs(timeIn1-ctime));
    cRise = !cRise;
    P3 -> OUT ^= 0x40;
}


void Right_Handler(int32_t timeIn3)
{
    if(rRise) rtime = timeIn3;
    else nr = LPF_Calc(abs(timeIn3-rtime));
    rRise = !rRise;
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
    TimerA2Capture_Init(&Center_Handler, &Right_Handler);
    EnableInterrupts();

}

//FUNCTION TO HANDLE THE BUMP SENSORS
void HandleCollision(uint8_t bump){
    if(bump != 0x3F)
    {
        //Stop the timer
        TimerA0_Stop();
        TimerA1_Stop();
        Port2_Output(BLUE);
        Motor_Backward(3000,3000);
        Clock_Delay1ms(750);
        TimerA0_Init(&Task, 500);
        TimerA1_Init(&Trigger_Handler, 65535);
    }

}

void PORT4_IRQHandler(void){
    P4 -> IFG &= 0x12; //clears pending flags
    HandleCollision(Bump_Read());
}

//SYSTICK HANDLER FOR MEASURING ULTRASONIC AND REFLECTANCE SENSORS

//Variables to store whether a wall is present or not
uint8_t FWall,RWall;
void SysTick_Handler(void){ // every 1ms
    if (Time % 5 == 0){
        //Start charging capacitors in reflectance sensor
        Reflectance_Start();
    }
    else if (Time % 5 == 1){
        //Gets the reading of the reflectance sensors
        LineReading = Reflectance_End();
    }

    //Get the converted values of center and right
    Center = CenterConvert(nc);
    Right = RightConvert(nr);

    //Determines if a wall is detected or not
    FWall = (Center > 0 ? 1 : 0);
    RWall = (Right  > 0 ? 1 : 0);

    //Display results to serial port
    if(Time % 1000 == 0){

        EUSCIA0_OutString("\nDebug MSG\n");

        EUSCIA0_OutString("\nRight\n");
        EUSCIA0_OutUDec(Right); EUSCIA0_OutChar(LF);
        EUSCIA0_OutUDec(nr); EUSCIA0_OutChar(LF);
        EUSCIA0_OutUDec(RWall); EUSCIA0_OutChar(LF);

        EUSCIA0_OutString("\nCenter\n");
        EUSCIA0_OutUDec(Center); EUSCIA0_OutChar(LF);
        EUSCIA0_OutUDec(nc); EUSCIA0_OutChar(LF);
        EUSCIA0_OutUDec(FWall); EUSCIA0_OutChar(LF);
    }
    Time++;
}

void Trigger_Init()
{
    //Initialize P3.5 as the trigger (output)
    P3 -> SEL0 &= ~0x60;
    P3 -> SEL1 &= ~0x60;
    P3 -> DIR  |= 0x60;
    P3 -> OUT  &= ~0x60;
}

void Navigate_Maze(){


    if(FWall == 0 && RWall == 1) //Straight
    {
        Port2_Output(RED);
        Travelling();
    }
    else if(FWall == 1 && RWall == 1) //Turn Left
    {
        Port2_Output(YELLOW);
        Turn_Left();
        Travelling();
    }
    else if(RWall == 0) //Turn Right
    {
        Port2_Output(GREEN);
        Turn_Right();
        Travelling();
    }
    /*else if(LineReading!= 0x3F){ // Treasure Found
           Found_Treasure();
           AMAZING LIGHT SHOW WITH LED
        }*/
    else
    {
        Travelling();
        //Port2_Output(PURPLE);
    }

}

// BLUETOOTH
// BLE variables
//uint8_t JackiCommand=0;
//uint16_t JackiSpeed=0;
//
//void OutValue(char *label,uint32_t value){
//  UART0_OutString(label);
//  UART0_OutUHex(value);
//}
//
//void ReadCommand(void){ // called on a SNP Characteristic Read Indication for characteristic JackiCommand
//  OutValue("\n\rRead JackiCommand=",JackiCommand);
//}
//
//void RunJacki(void){
//  if((JackiCommand==0)||(JackiCommand>4)){
//    JackiCommand = 0;
//    Motor_Stop();
//  }
//  if(JackiSpeed>14000){
//    JackiSpeed = 1000;
//  }
//  if(JackiCommand==1){
//    Motor_Forward(JackiSpeed,JackiSpeed);
//  }
//  if(JackiCommand==2){
//    Motor_Backward(JackiSpeed/2,JackiSpeed/2);
//    //time=0;
//  }
//  if(JackiCommand==3){
//    Motor_Right(JackiSpeed/2,JackiSpeed/2);
//    //time=0;
//  }
//  if(JackiCommand==4){
//    Motor_Left(JackiSpeed/2,JackiSpeed/2);
//    //time=0;
//  }
//}
//
//void WriteCommand(void){ // called on a SNP Characteristic Write Indication on characteristic JackiCommand
//  OutValue("\n\rWrite JackiCommand=",JackiCommand);
//  RunJacki();
//}
//
//void ReadJackiSpeed(void){ // called on a SNP Characteristic Read Indication for characteristic JackiSpeed
//  OutValue("\n\rRead JackiSpeed=",JackiSpeed);
//}
//
//void WriteJackiSpeed(void){  // called on a SNP Characteristic Write Indication on characteristic JackiSpeed
//  OutValue("\n\rJackiSpeed=",JackiSpeed);
//  RunJacki();
//}
//
//void BLE_Init(void){
//    volatile int r;
//    UART0_Init();
//    EnableInterrupts();
//    UART0_OutString("\n\rJacki test project - MSP432-CC2650\n\r");
//    r = AP_Init();
//    AP_AddService(0xFFF0);
//    //------------------------
//    JackiCommand = 0;  // read/write parameter
//    AP_AddCharacteristic(0xFFF1,1,&JackiCommand,0x03,0x0A,"JackiCommand",&ReadCommand,&WriteCommand);
//    //------------------------
////    JackiLineSensor = Reflectance_End(); // read only parameter (get from line sensors)
////    AP_AddCharacteristic(0xFFF2,1,&JackiLineSensor,0x01,0x02,"JackiLineSensor",&ReadJackiLineSensor,0);
//    //------------------------
//    JackiSpeed = 100;   // read/write parameter
//    AP_AddCharacteristic(0xFFF3,2,&JackiSpeed,0x03,0x0A,"JackiSpeed",&ReadJackiSpeed,&WriteJackiSpeed);
//    //------------------------
//    AP_RegisterService();
//    AP_StartAdvertisementJacki();
//}

uint8_t prev = 0;
void main(void){

    //Disable interrupts while we configure everything
    DisableInterrupts();

    //Initialize the clock
    Clock_Init48MHz();

    //Low pass filter initializers
    LPF_Init(20, 5); //Right
    LPF_Init2(0, 5); //Center

    //Configure the trigger pin that drives the ultrasonic sensors
    Trigger_Init();

    //Initialize the bump sensors (although they will not be used)
    Bump_Init(&HandleCollision);

    TimerA0_Init(&Task, 500);

    //Initliaze the timer that handles the motors
    TimerA1_Init(&Trigger_Handler, 65535);

    //Intialize the timer for Wall Sensors (5.6 = center, 5.7 = right)
    TimerA2Capture_Init(&Center_Handler, &Right_Handler);

    //Enable UART Comms
    EUSCIA0_Init();

    //Intialize time and LineReading to be 0
    Time = LineReading = 0;

    //Initliaze Systick to have a frequency of 10 us
    SysTick_Init(48000, 0);

    //Initialize the reflectance sensors
    Reflectance_Init();

    //Initialize port 2 for LED Debugging and FoundTreasure functionality
    Port2_Init();

    //Initialize the motors
    Motor_Init();

    //Initialize the tachometer for cleaner turns
    Tachometer_Init();

    FWall = 0;
    RWall = 1;

    // BLE
    //BLE_Init();

    //Enable interrupts
    EnableInterrupts();

    while(1){

        //Navigate the maze
        Travelling();
        Navigate_Maze();

        //Wait for interrupts to come
        WaitForInterrupt();

    }
}

