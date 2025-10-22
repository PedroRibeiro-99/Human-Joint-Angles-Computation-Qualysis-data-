#include "../include/qualysis_data/rula.h"


int tableA[18][8] = { 1,2,2,2,2,3,3,3,
                     2,2,2,2,3,3,3,3,
                     2,3,3,3,3,3,4,4,

                     2,3,3,3,3,4,4,4,
                     3,3,3,3,3,4,4,4,
                     3,4,4,4,4,4,5,5,

                     3,3,4,4,4,4,5,5,
                     3,4,4,4,4,4,5,5,
                     4,4,4,4,4,5,5,5,

                     4,4,4,4,4,5,5,5,
                     4,4,4,4,4,5,5,5,
                     4,4,4,5,5,5,6,6,

                     5,5,5,5,5,6,6,7,
                     5,6,6,6,6,7,7,7,
                     6,6,6,7,7,7,7,8,

                     7,7,7,7,7,8,8,9,
                     8,8,8,8,8,9,9,9,
                     9,9,9,9,9,9,9,9 };


int tableB[6][12] = { 1,3,2,3,3,4,5,5,6,6,7,7,
                     2,3,2,3,4,5,5,5,6,7,7,7,
                     3,3,3,4,4,5,5,6,6,7,7,7,
                     5,5,5,6,6,7,7,7,7,7,8,8,
                     7,7,7,7,7,8,8,8,8,8,8,8,
                     8,8,8,8,8,8,8,9,9,9,9,9 };

int tableC[8][7] = { 1,2,3,3,4,5,5,
                    2,2,3,4,4,5,5,
                    3,3,3,4,4,5,6,
                    3,3,3,4,5,6,6,
                    4,4,4,5,6,7,7,
                    4,4,5,6,6,7,7,
                    5,5,6,6,7,7,7,
                    5,5,6,7,7,7,7 };


int upper_lower_arm_index[6][3] = { 0,1,2,
                                    3,4,5,
                                    6,7,8,
                                    9,10,11,
                                    12,13,14,
                                    15,16,17 };

int wrist_twist_index[4][2] = { 0,1,
                                2,3,
                                4,5,
                                6,7 };

int trunk_legs_index[6][2] = { 0,1,
                               2,3,
                               4,5,
                               6,7,
                               8,9,
                               10,11};

int upperArmEvaluationStatus[6] = {GOOD, GOOD, MODERATED, MODERATED, BAD, BAD};
int lowerArmEvaluationStatus[3] = {GOOD, MODERATED, BAD};
int wristEvaluationStatus[4] = {GOOD, GOOD, MODERATED, BAD};
int neckEvaluationStatus[6] = {GOOD, GOOD, MODERATED, MODERATED, BAD, BAD};
int trunkEvaluationStatus[6] = {GOOD, GOOD, MODERATED, MODERATED, BAD, BAD};


Rula::Rula()
{
}

void Rula::setRULA_Variables(RULA_SegmentsVariables &rulaVariables){
  this->upperArm = rulaVariables.upperArm;
  this->lowerArm = rulaVariables.lowerArm;
  this->wrist = rulaVariables.wrist;
  this->neck = rulaVariables.neck;
  this->trunk = rulaVariables.trunk;
  this->legs = rulaVariables.legs;
  this->loadScore = rulaVariables.loadScore;
}

void Rula::computeUpperArmEvaluation(){

   upperArm.evaluation = 1;

   if(upperArm.flexion >= -20 && upperArm.flexion <= 20) upperArm.evaluation = 1;
   else if((upperArm.flexion > 20 && upperArm.flexion <= 45) || (upperArm.flexion < -20)) upperArm.evaluation = 2;
   else if(upperArm.flexion > 45 && upperArm.flexion <= 90) upperArm.evaluation = 3;
   else if(upperArm.flexion > 90) upperArm.evaluation = 4;

   upperArm.evaluation += (upperArm.raised + upperArm.abducted);
   upperArm.evaluation_status = upperArmEvaluationStatus[upperArm.evaluation - 1];
}

void Rula::computeLowerArmEvaluation(){

   lowerArm.evaluation = 1;

   if(lowerArm.flexion > 60 && lowerArm.flexion <= 100) lowerArm.evaluation = 1;
   else if(lowerArm.flexion < 0 || (lowerArm.flexion >= 0 && lowerArm.flexion <= 60) || (lowerArm.flexion > 100)) lowerArm.evaluation = 2;

   lowerArm.evaluation += lowerArm.body_midline_exceeded;
   lowerArm.evaluation_status = lowerArmEvaluationStatus[lowerArm.evaluation - 1];

}

void Rula::computeWristEvaluation(){

   wrist.evaluation = 1;

   if(wrist.flexion >= -1 && wrist.flexion <= 1) wrist.evaluation = 1;
   else if(wrist.flexion >= -15 && wrist.flexion <= 15) wrist.evaluation = 2;
   else if(wrist.flexion < -15 || wrist.flexion > 15) wrist.evaluation = 3;

   wrist.evaluation += wrist.twisted + wrist.deviated; //midline_exceeded;
   wrist.evaluation_status = wristEvaluationStatus[wrist.evaluation - wrist.twisted - 1];
}




void Rula::computeNeckEvaluation(){

  neck.evaluation = 1;

  if(neck.flexion >= -5 && neck.flexion <=10) neck.evaluation = 1;
  else if(neck.flexion > 10 && neck.flexion <= 20) neck.evaluation = 2;
  else if(neck.flexion > 20) neck.evaluation = 3;
  else if(neck.flexion < -5) neck.evaluation = 4;

  neck.evaluation += neck.bent + neck.twisted;
  neck.evaluation_status = neckEvaluationStatus[neck.evaluation - 1];
}

void Rula::computeTrunkEvaluation(){

  trunk.evaluation = 1;

  switch(trunk.standing){
  case true:
    if(trunk.flexion >= -5 && trunk.flexion <= 0.1) trunk.evaluation = 1;
    else if(trunk.flexion > 0.1 && trunk.flexion <= 20) trunk.evaluation = 2;
    else if(trunk.flexion > 20 && trunk.flexion <= 60) trunk.evaluation = 3;
    else if(trunk.flexion > 60) trunk.evaluation = 4;
  break;
  case false:
    if(trunk.well_suported) trunk.evaluation = 1;
    else trunk.evaluation = 2;
  break;
  }

  trunk.evaluation += trunk.bent + trunk.twisted;
  trunk.evaluation_status = trunkEvaluationStatus[trunk.evaluation - 1];
}

void Rula::computeLegsEvaluation(){

  legs.evaluation = legs.stable;
}


void Rula::computeGroupA_Evaluation(){

  int rowIndex = -1;
  int columnIndex = -1;

  rowIndex = upper_lower_arm_index[upperArm.evaluation-1][lowerArm.evaluation-1];
  columnIndex = wrist_twist_index[(wrist.evaluation-wrist.twisted)-1][wrist.twisted-1];

  groupA_Score = tableA[rowIndex][columnIndex];
  groupA_Score += loadScore;
}

void Rula::computeGroupB_Evaluation(){

  int rowIndex = neck.evaluation - 1;
  int columnIndex = trunk_legs_index[trunk.evaluation-1][legs.evaluation-1];

  groupB_Score = tableB[rowIndex][columnIndex];
  groupB_Score += loadScore;
}

void Rula::computeTotalEvaluation(){

  int rowIndex = groupA_Score - 1;
  int columnIndex = groupB_Score - 1;

  if(rowIndex > 7) rowIndex = 7;
  if(columnIndex > 6) columnIndex = 6;

  totalScore = tableC[rowIndex][columnIndex];

}

void Rula::executeRulaEvalutation(){

  this->computeUpperArmEvaluation();
  this->computeLowerArmEvaluation();
  this->computeWristEvaluation();
  this->computeNeckEvaluation();
  this->computeTrunkEvaluation();
  this->computeLegsEvaluation();
  this->computeGroupA_Evaluation();
  this->computeGroupB_Evaluation();
  this->computeTotalEvaluation();
}

int Rula::getUpperArmScore(){
  return this->upperArm.evaluation;
}


int Rula::getLowerArmScore(){
  return this->lowerArm.evaluation;
}


int Rula::getWristScore(){
  return this->wrist.evaluation;
}

int Rula::getWristTwistScore(){
  return this->wrist.twisted;
}


int Rula::getNeckScore(){
  return this->neck.evaluation;
}

int Rula::getTrunkScore(){
  return this->trunk.evaluation;
}

int Rula::getLegsScore(){
  return this->legs.evaluation;
}

int Rula::getGroupA_Score(){
  return this->groupA_Score;
}

int Rula::getGroupB_Score(){
  return this->groupB_Score;
}

int Rula::getTotalScore(){
  return this->totalScore;
}

int Rula::getUpperArmScoreStatus(){
  return this->upperArm.evaluation_status;
}


int Rula::getLowerArmScoreStatus(){
  return this->lowerArm.evaluation_status;
}


int Rula::getWristScoreStatus(){
  return this->wrist.evaluation_status;
}


int Rula::getNeckScoreStatus(){
  return this->neck.evaluation_status;
}

int Rula::getTrunkScoreStatus(){
  return this->trunk.evaluation_status;
}
