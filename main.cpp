#include "vex.hpp"
#include <math.h>
using namespace vex;
motor LeftDrive(4);
motor RightDrive(1);
controller Controller1(primary);
motor RightLift(8, ratio18_1, false);
motor LeftLift(9, ratio18_1, true);
int scroll = 0;
int cursor = 1;
int motorLooking = -1;
void buttonUp(){
  if(cursor>1)cursor--;
  else if(scroll>0)scroll--;
}
void buttonDown(){
    if(cursor<3)cursor++;
    else scroll++;
}
int main() {
  bool driveLocked = false;
  RightLift.setPosition(0,degrees);
  LeftLift.setPosition(0,degrees);
  int rDest = 0;
  int lDest = 0;
  int maxLiftAngle = 742;
  int minLiftAngle = -247;
  int startFalloff = 30*5;
  double minStrength = 0.3;
  double endFalloff = (startFalloff*minStrength)/(minStrength-1);
  while(true){
    Debug.debug();
    if(!driveLocked){
      //math
      if(Controller1.ButtonLeft.pressing()){
        if(lDest == maxLiftAngle-20)lDest = 0;
        else lDest = maxLiftAngle-20;
      }
      if(Controller1.ButtonRight.pressing()){
        if(rDest == minLiftAngle-20)rDest = 0;
        else rDest = minLiftAngle-20;
      }
      //reset button
      if(Controller1.ButtonA.pressing()){
        lDest = 0;
        rDest = 0;
      }
      double lMod = ((std::abs(LeftLift.position(degrees)))-(endFalloff+lDest))/((startFalloff+lDest)-(endFalloff+lDest));
      double rMod = ((std::abs(RightLift.position(degrees)))-(endFalloff+rDest))/((startFalloff+rDest)-(endFalloff+rDest));
      if(lMod>1)lMod=1;
      if(rMod>1)rMod=1;
      double rSpin = 0;
      double lSpin = 0;
      if(RightLift.position(degrees)>rDest)rSpin = -rMod;
      else rSpin = rMod;
      if(LeftLift.position(degrees)>lDest)lSpin = -lMod;
      else lSpin = lMod;
      if(RightLift.position(degrees)>=rDest-20 && RightLift.position(degrees)<=rDest+20)rSpin = 0;
      if(LeftLift.position(degrees)>=rDest-20 && LeftLift.position(degrees)<=rDest+20)rSpin = 0;
      //controller overrides R1 and R2
      if(Controller1.ButtonR1.pressing()){
        if(RightLift.position(degrees)>minLiftAngle)rSpin = -0.7;
        if(LeftLift.position(degrees)>minLiftAngle)lSpin = -0.7;
        lDest = LeftLift.position(degrees);
        rDest = RightLift.position(degrees);
      }
      if(Controller1.ButtonR2.pressing()){
        if(RightLift.position(degrees)<maxLiftAngle)rSpin = 0.7;
        if(LeftLift.position(degrees)<maxLiftAngle)lSpin = 0.7;
        lDest = LeftLift.position(degrees);
        rDest = RightLift.position(degrees);
      }
      LeftLift.spin(forward,lSpin*70,pct);
      RightLift.spin(forward,rSpin*70,pct);
      LeftDrive.spin(forward,Controller1.Axis3.value(),pct);
      RightDrive.spin(forward,Controller1.Axis2.value(),pct);
      if(driveLocked){
        RightLift.stop();
        LeftLift.stop();
        RightDrive.stop();
        LeftDrive.stop();
      }
    }
    }
}