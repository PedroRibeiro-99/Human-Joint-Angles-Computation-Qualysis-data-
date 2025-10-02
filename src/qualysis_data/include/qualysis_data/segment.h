#ifndef SEGMENT_H
#define SEGMENT_H

#include "qualysis_move_data.h"
#include <eigen3/Eigen/Dense>
#include <string>

using namespace std;
using namespace Eigen;

typedef struct CoordinateRefFrame{
  Vector3d xVector = {0,0,0};
  Vector3d yVector = {0,0,0};
  Vector3d zVector = {0,0,0};
  Vector3d position = {0,0,0};
  Vector3d eulerAngles = {0,0,0}; //X-Y-Z
};

void getRPY(Vector3d &eulerAngles, Matrix3d Rot);
void getRPY_YZX(Vector3d &eulerAngles, Matrix3d Rot);
void getRPY_XYZ(Vector3d &eulerAngles, Matrix3d Rot);


class Segment
{
private:
  string name;
  vector<int> markersId;
  //Base Reference Frames results from creating a plane formed by the markers that represent the segment
  CoordinateRefFrame baseRefFrame;
  int normDirection;
  int dirDirection;
  //Calibrated Reference frame results from a rotation of the Base Reference Frame in order to respect the standard reference frames for the neutral posture
  CoordinateRefFrame calibratedRefFrame;
  Vector3d calibrationAngles;
  bool validData; //false if the coordinates of a market element is (0,0,0), true if all the markers present valid coordinates

public:
  Segment();
  void setName(string name);
  string getName();
  void initSegment(vector<int> markersId, int normDirection, int dirDirection, Vector3d calibrationAngles);
  void resetData();
  void getMarkersNames(vector<string> &markers_str);
  bool verifyData(QualysisMoveData &q_obj,int frame);
  bool getDataStatus();
  void computeNormalVector(QualysisMoveData &q_obj, int frame);
  void computeDirectorVector(QualysisMoveData &q_obj, int frame);
  void computeYVector();
  void computePosition(QualysisMoveData &q_obj, int frame);
  void computeBaseRefFrameRPY();
  void computeBaseRefFrame(QualysisMoveData &q_obj, int frame);
  void computeCalibratedRefFrame();
  void getBaseRefFrame(CoordinateRefFrame &baseRefFrame);
  void getCalibratedRefFrame(CoordinateRefFrame &calibratedRefFrame);
};

#endif // SEGMENT_H
