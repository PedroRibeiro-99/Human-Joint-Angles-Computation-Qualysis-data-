#ifndef JOINT_H
#define JOINT_H

#include "segment.h"
#include <math.h>

class Joint{
private:
  Vector3d eulerAngles;
  vector<string> markersNames;
  Segment *segment1, *segment2;
  string name;
  bool validData;

public:
  Joint();
  void initJoint(string name, Segment *segment1, Segment *segment2);
  void resetData();
  bool verifyData();
  bool getDataStatus();
  string getName();
  void computeEulerAngles();
  void getEulerAngles(Vector3d &eulerAngles);
};

#endif // JOINT_H
