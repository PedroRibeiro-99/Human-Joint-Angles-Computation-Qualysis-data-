#ifndef RULAINTERFACE_H
#define RULAINTERFACE_H

#include "joint.h"
#include "math.h"

typedef struct{
  float flexion = 0;
  int raised = 0;
  int abducted = 0;
  bool arm_weight_supported = false;
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
  int bent = 0;
  int evaluation = 0;
  int evaluation_status = 0;
}TrunkVariables;

typedef struct{
  int stable = 0;
  int evaluation = 0;
}LegsVariables;

typedef struct{
  UpperArmVariables upperArm;
  LowerArmVariables lowerArm;
  WristVariables wrist;
  NeckVariables neck;
  TrunkVariables trunk;
  LegsVariables legs;
  int loadScore = 0;
}RULA_SegmentsVariables;

void setUpperArmVariables(Joint &jArm, UpperArmVariables &upperArm);
void setLowerArmVariables(Joint &jForeArm, LowerArmVariables &lowerArm);
void setWristVariables(Joint &jWrist, WristVariables &wrist);
void setNeckVariables(Joint &jNeck, NeckVariables &neck);
void setTrunkVariables(Joint &jTrunk, TrunkVariables &trunk);
void setLegsVariables(LegsVariables &legs);
void setRULA_Variables(vector<Joint> &joints, RULA_SegmentsVariables &rulaVariables);

#endif // RULAINTERFACE_H
