// FIFO0.c
// Runs on any microcontroller
// Two first in first out queues
// These will be implemented as part of Lab 18
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
#include "..\inc\FIFO0.h"

// Implementation of the transmit FIFO, TxFifo0
// can hold 0 to TXFIFOSIZE-1 elements
// you are allowed to restrict TXFIFOSIZE to a power of 2

// add static, volatile, global variables here this as part of Lab 18
static volatile uint8_t TxBuff[TX0FIFOSIZE];
static volatile uint8_t TxStart;
static volatile uint8_t TxEnd;
static volatile uint8_t RxBuff[Rx0FIFOSIZE];
static volatile uint8_t RxStart;
static volatile uint8_t RxEnd;

uint32_t TxHistogram[TX0FIFOSIZE];
// probability mass function of the number of times TxFifo0 as this size
// as a function of FIFO size at the beginning of call to TxFifo0_Put
// initialize index TxFifo0
void TxFifo0_Init(void){int i;


  for(i=0;i<TX0FIFOSIZE;i++){
      TxBuff[i] = 0;
  }

  for(i=0;i<TX0FIFOSIZE;i++){
      TxHistogram[i] = 0;
  }

  TxStart = 0;
  TxEnd = 0;
}
// add element to end of index TxFifo0
// return TXFIFOSUCCESS if successful
int TxFifo0_Put(char data){
  TxHistogram[TxFifo0_Size()]++;  // probability mass function
  if(TxFifo0_Size()==TX0FIFOSIZE -1 )
      return FIFOFAIL;
  TxBuff[TxEnd]=data;
  TxEnd= (TxEnd+1) & (TX0FIFOSIZE -1);



// write this as part of Lab 18

  return(FIFOSUCCESS);
}
// remove element from front of TxFifo0
// return TXFIFOSUCCESS if successful
int TxFifo0_Get(char *datapt){

    if(TxFifo0_Size()==0 )
         return FIFOFAIL;
     *datapt = TxBuff[TxStart];
     TxStart= (TxStart+1) & (TX0FIFOSIZE -1);
     return(FIFOSUCCESS);
}
// number of elements in TxFifo0
// 0 to TXFIFOSIZE-1
uint16_t TxFifo0_Size(void){
    if(TxStart <= TxEnd){
        return TxEnd - TxStart;
    }else{
        return TxEnd - TxStart + TX0FIFOSIZE;
    }
}

// Implementation of the receive FIFO, RxFifo0
// can hold 0 to RXFIFOSIZE-1 elements
// you are allowed to restrict RXFIFOSIZE to a power of 2
uint32_t RxHistogram[Rx0FIFOSIZE];
// probability mass function of the number of times RxFifo0 as this size
// as a function of FIFO size at the beginning of call to RxFifo0_Put
// initialize index RxFifo0
void RxFifo0_Init(void){int i;


  for(i=0;i<Rx0FIFOSIZE;i++){
      RxBuff[i] = 0;
  }

  for(i=0;i<Rx0FIFOSIZE;i++){
      RxHistogram[i] = 0;
  }

  RxStart = 0;
  RxEnd = 0;
}
// add element to end of index RxFifo0
// return RxFIFOSUCCESS if successful
int RxFifo0_Put(char data){
  RxHistogram[RxFifo0_Size()]++;  // probability mass function
  if(RxFifo0_Size()==Rx0FIFOSIZE -1 )
      return FIFOFAIL;
  RxBuff[RxEnd]=data;
  RxEnd= (RxEnd+1) & (Rx0FIFOSIZE -1);



// write this as part of Lab 18

  return(FIFOSUCCESS);
}
// remove element from front of RxFifo0
// return RxFIFOSUCCESS if successful
int RxFifo0_Get(char *datapt){

    if(RxFifo0_Size()==0 )
         return FIFOFAIL;
     *datapt = RxBuff[RxStart];
     RxStart= (RxStart+1) & (Rx0FIFOSIZE -1);
     return(FIFOSUCCESS);
}
// number of elements in RxFifo0
// 0 to RxFIFOSIZE-1
uint16_t RxFifo0_Size(void){
    if(RxStart <= RxEnd){
        return RxEnd - RxStart;
    }else{
        return RxEnd - RxStart + Rx0FIFOSIZE;
    }
}

