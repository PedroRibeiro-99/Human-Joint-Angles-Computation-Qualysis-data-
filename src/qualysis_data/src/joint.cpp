#include "../include/qualysis_data/joint.h"

Joint::Joint(){}

void Joint::initJoint(string name, Segment *segment1, Segment *segment2){
  this->name = name;
  this->segment1 = segment1;
  this->segment2 = segment2;
}

void Joint::resetData(){
  this->eulerAngles.x() = 0;
  this->eulerAngles.y() = 0;
  this->eulerAngles.z() = 0;
}

bool Joint::verifyData(){
  if(this->segment1->getDataStatus() && this->segment2->getDataStatus())
    this->validData = true;
  else
    this->validData = false;

  return this->validData;
}

bool Joint::getDataStatus(){
  return this->validData;
}

string Joint::getName(){
  return this->name;
}

void Joint::computeEulerAngles(){
  Matrix3d segment1Matrix, segment2Matrix;

  CoordinateRefFrame segment1RefFrame,segment2RefFrame;
  this->segment1->getCalibratedRefFrame(segment1RefFrame);
  this->segment2->getCalibratedRefFrame(segment2RefFrame);

  segment1Matrix.col(0) = segment1RefFrame.xVector.normalized();
  segment1Matrix.col(1) = segment1RefFrame.yVector.normalized();
  segment1Matrix.col(2) = segment1RefFrame.zVector.normalized();

  segment2Matrix.col(0) = segment2RefFrame.xVector.normalized();
  segment2Matrix.col(1) = segment2RefFrame.yVector.normalized();
  segment2Matrix.col(2) = segment2RefFrame.zVector.normalized();

  Matrix3d rotMatrix;
  rotMatrix = segment1Matrix.transpose() * segment2Matrix;

  getRPY_XYZ(this->eulerAngles,rotMatrix);
  this->eulerAngles *= 180/3.1415;
}

void Joint::getEulerAngles(Vector3d &eulerAngles){
  eulerAngles = this->eulerAngles;
}
