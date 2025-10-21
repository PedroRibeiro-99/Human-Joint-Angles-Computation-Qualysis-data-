#include "../include/qualysis_data/rulainterface.h"

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
  Vector3d wristAngles;
  jWrist.getEulerAngles(wristAngles);

  wrist.flexion = wristAngles.y();

  if(abs(wristAngles.z()) > 95) wrist.twisted = 2;
  else wrist.twisted = 1;

  if(abs(wristAngles.x()) > 10) wrist.deviated = 1;
  else wrist.deviated = 0;
}

void setNeckVariables(Joint &jNeck, NeckVariables &neck){
  Vector3d neckAngles;
  jNeck.getEulerAngles(neckAngles);

  neck.flexion = neckAngles.x();

  if(abs(neckAngles.z()) > 10) neck.twisted = 1;
  else neck.twisted = 0;

  if(abs(neckAngles.y()) > 10) neck.bent = 1;
  else neck.bent = 0;
}

void setTrunkVariables(Joint &jTrunk, TrunkVariables &trunk){
  Vector3d trunkAngles;
  jTrunk.getEulerAngles(trunkAngles);

  trunk.flexion = trunkAngles.x();

  if(abs(trunkAngles.z()) > 10) trunk.twisted = 1;
  else trunk.twisted = 0;

  if(abs(trunkAngles.y()) > 10) trunk.bent = 1;
  else trunk.bent = 0;

  trunk.standing = false;
  trunk.well_suported = true;
}

void setLegsVariables(LegsVariables &legs){
  legs.stable = 1;
}

void setRULA_Variables(vector<Joint> &joints, RULA_SegmentsVariables &rulaVariables){
  setUpperArmVariables(joints.at(0), rulaVariables.upperArm);
  setLowerArmVariables(joints.at(1), rulaVariables.lowerArm);
  setWristVariables(joints.at(2), rulaVariables.wrist);
  setNeckVariables(joints.at(3), rulaVariables.neck);
  setTrunkVariables(joints.at(4), rulaVariables.trunk);
  setLegsVariables(rulaVariables.legs);
}
