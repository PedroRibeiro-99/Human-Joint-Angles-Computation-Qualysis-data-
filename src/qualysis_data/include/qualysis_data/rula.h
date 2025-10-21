#ifndef RULA_H
#define RULA_H

#include "rulainterface.h"

#define GOOD 1
#define MODERATED 2
#define BAD 3


using namespace std;


class Rula
{
private:
  UpperArmVariables upperArm;
  LowerArmVariables lowerArm;
  WristVariables wrist;
  NeckVariables neck;
  TrunkVariables trunk;
  LegsVariables legs;
  int loadScore;
  int groupA_Score;
  int groupB_Score;
  int totalScore;

public:
  Rula();
  void setRULA_Variables(RULA_SegmentsVariables &rulaVariables);
  void computeUpperArmEvaluation();
  void computeLowerArmEvaluation();
  void computeWristEvaluation();
  void computeNeckEvaluation();
  void computeTrunkEvaluation();
  void computeLegsEvaluation();
  void computeGroupA_Evaluation();
  void computeGroupB_Evaluation();
  void computeTotalEvaluation();
  void executeRulaEvalutation();
  int getUpperArmScore();
  int getLowerArmScore();
  int getWristScore();
  int getWristTwistScore();
  int getNeckScore();
  int getTrunkScore();
  int getLegsScore();
  int getGroupA_Score();
  int getGroupB_Score();
  int getTotalScore();

  int getUpperArmScoreStatus();
  int getLowerArmScoreStatus();
  int getWristScoreStatus();
  int getNeckScoreStatus();
  int getTrunkScoreStatus();
};

#endif // RULA_H
