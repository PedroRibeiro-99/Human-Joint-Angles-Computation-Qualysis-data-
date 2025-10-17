#ifndef RULAINTERFACE_H
#define RULAINTERFACE_H

#include "joint.h"
#include "math.h"

typedef struct{
  float flexion = 0;
  int raised = 0;
  int abducted = 0;
  int arm_weight_supported = 0;
  int evaluation = 0;
  int evaluation_status = 0;
}UpperArmVariables;

typedef struct{
  float flexion = 0;
  int body_midline_exceeded = 0;
  int evaluation = 0;
  int evaluation_status = 0;
}LowerArmVariables;

typedef struct{
  float flexion = 0;
  int deviated = 0;
  int twisted = 0;
  int evaluation = 0;
  int evaluation_status = 0;
}WristVariables;

typedef struct{
  float flexion = 0;
  int twisted = 0;
  int bent = 0;
  int evaluation = 0;
  int evaluation_status = 0;
}NeckVariables;

typedef struct{
  bool standing = 0;
  bool well_suported = 0;
  float flexion = 0;
  int twisted = 0;
  int bending = 0;
  int evaluation = 0;
  int evaluation_status = 0;
}TrunkVariables;

typedef struct{
  int stable = 0;
  int evaluation = 0;
}LegsVariables;

void setUpperArmVariables(Joint &jArm, UpperArmVariables &upperArm){
  Vector3d armAngles;
  jArm.getEulerAngles(armAngles);

  upperArm.flexion = armAngles.x();

  if(abs(armAngles.y()) > 10) upperArm.abducted = 1;
  else upperArm.abducted = 0;

  upperArm.raised = 0;
  upperArm.arm_weight_supported = 0;
}

void setLowerArmVariables(Joint &jForeArm, LowerArmVariables &lowerArm){
  Vector3d foreArmAngles;
  jForeArm.getEulerAngles(foreArmAngles);

  lowerArm.flexion = foreArmAngles.x();

  if(abs(foreArmAngles.z()) > 10) lowerArm.body_midline_exceeded = 1;
  else lowerArm.body_midline_exceeded = 0;
}

void setWristVariables(Joint &jWrist, WristVariables &wrist){

}

void setNeckVariables(Joint &jNeck, NeckVariables &neck){

}

void setTrunkVariables(Joint &jTrunk, TrunkVariables &trunk){

}

void setLegsVariables(LegsVariables &legs){

}

#endif // RULAINTERFACE_H
