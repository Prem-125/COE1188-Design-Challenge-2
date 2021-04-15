/*
 * solution.c
 *
 *  Created on: May 22, 2020
 *      Author: ValvanoJonathan
 */

#include <stdint.h>

enum scenario {
    Error = 0,
    LeftTooClose = 1,
    RightTooClose = 2,
    CenterTooClose = 4,
    Straight = 8,
    LeftTurn = 9,
    RightTurn = 10,
    TeeJoint = 11,
    LeftJoint = 12,
    RightJoint = 13,
    CrossRoad = 14,
    Blocked = 15
};
typedef enum scenario scenario_t;

#define SIDEMAX 354    // largest side distance to wall in mm
#define SIDEMIN 212    // smallest side distance to wall in mm
#define CENTEROPEN 600 // distance to wall between open/blocked
#define CENTERMIN 150  // min distance to wall in the front
scenario_t Solution(int32_t Left, int32_t Center, int32_t Right){
  scenario_t result=Error;
  if((Center<50)||(Center>800)) return Error;
  if((Left<50)||(Left>800)) return Error;
  if((Right<50)||(Right>800)) return Error;
  if(Center < CENTERMIN){
    result |= CenterTooClose;
  }
  if(Left < SIDEMIN){
    result |=  LeftTooClose;
  }
  if(Right < SIDEMIN){
    result |=  RightTooClose;
  }
  if(result){  // 1 to 7?
    return result; // too close to a wall
  }
// CrossRoad if (Left is greater than or equal to SIDEMAX)
// and (Right is greater than or equal to SIDEMAX)
// and (Center is greater than or equal to  CENTEROPEN)
  if((Left>=SIDEMAX)&&(Right>=SIDEMAX)&&(Center>=CENTEROPEN)){
    return CrossRoad;
  }

// RightJoint if (Left is between SIDEMIN and SIDEMAX)
// and (Right is greater than or equal to SIDEMAX)
// and (Center is greater than or equal to CENTEROPEN)
  if((Left<SIDEMAX)&&(Right>=SIDEMAX)&&(Center>=CENTEROPEN)){
    return RightJoint;
  }
// LeftJoint if (Left is greater than SIDEMAX)
// and (Right is between SIDEMIN and SIDEMAX)
// and (Center is is greater than )
  if((Left>=SIDEMAX)&&(Right<SIDEMAX)&&(Center>=CENTEROPEN)){
    return LeftJoint;
  }
// TeeJoint if (Left is greater than or equal to SIDEMAX)
// and (Right is greater than or equal to SIDEMAX)
// and (Center is between CENTERMIN and CENTEROPEN)
  if((Left>=SIDEMAX)&&(Right>=SIDEMAX)&&(Center<CENTEROPEN)){
    return TeeJoint;
  }
// RightTurn if (Left is between SIDEMIN and SIDEMAX)
// and (Right is greater than or equal to SIDEMAX)
// and (Center is between CENTERMIN and CENTEROPEN)
  if((Left<SIDEMAX)&&(Right>=SIDEMAX)&&(Center<CENTEROPEN)){
    return RightTurn;
  }
// LeftTurn if (Left is greater than SIDEMAX)
// and (Right is between SIDEMIN and SIDEMAX)
// and (Center is between CENTERMIN and CENTEROPEN)
  if((Left>=SIDEMAX)&&(Right<SIDEMAX)&&(Center<CENTEROPEN)){
    return LeftTurn;
  }
// Blocked if (Left is between SIDEMIN and SIDEMAX)
// and (Right is between SIDEMIN and SIDEMAX)
// and (Center is between CENTERMIN and FRONTOPEN)
  if((Left<SIDEMAX)&&(Right<SIDEMAX)&&(Center<CENTEROPEN)){
    return Blocked;
  }
// straight if (Left is between SIDEMIN and SIDEMAX)
// and (Right is between SIDEMIN and SIDEMAX)
 // and (Center is greater than CENTEROPEN)
  if((Left<SIDEMAX)&&(Right<SIDEMAX)&&(Center>=CENTEROPEN)){
    return Straight;
  }
  return Error; // should happen
}

#define IRSlope 1195172
#define IROffset -1058
#define IRMax 2552

int32_t ConvertSolution(int32_t n){
  if(n < IRMax){
    return 800;
  }
  return IRSlope/(n+IROffset);
}





