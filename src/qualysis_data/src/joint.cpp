#include "../include/qualysis_data/joint.h"

Joint::Joint(){}

void Joint::initJoint(string name, vector<Segment*> segments){
  this->name = name;
  this->segments = segments;
}


void Joint::resetData(){
  this->eulerAngles.x() = 0;
  this->eulerAngles.y() = 0;
  this->eulerAngles.z() = 0;
}

bool Joint::verifyData(){
  this->validData = true;
  for(Segment *segment: segments){
    if(!(segment->getDataStatus())){
      this->validData = false;
      break;
    }
  }
  return this->validData;
}

bool Joint::getDataStatus(){
  return this->validData;
}

string Joint::getName(){
  return this->name;
}

void Joint::computeEulerAngles(){

  Matrix3d Matrix1, Matrix2;
  CoordinateRefFrame RefFrame1, RefFrame2;

  if(segments.size() == 1){
    Matrix1 = Matrix3d::Identity(); //World Reference Frame Matrix
    Segment *segment = this->segments.at(0);
    segment->getCalibratedRefFrame(RefFrame2);
    Matrix2.col(0) = RefFrame2.xVector.normalized();
    Matrix2.col(1) = RefFrame2.yVector.normalized();
    Matrix2.col(2) = RefFrame2.zVector.normalized();
  }
  else{
    Segment *segment1 = this->segments.at(0);
    Segment *segment2 = this->segments.at(1);

    segment1->getCalibratedRefFrame(RefFrame1);
    segment2->getCalibratedRefFrame(RefFrame2);

    Matrix1.col(0) = RefFrame1.xVector.normalized();
    Matrix1.col(1) = RefFrame1.yVector.normalized();
    Matrix1.col(2) = RefFrame1.zVector.normalized();

    Matrix2.col(0) = RefFrame2.xVector.normalized();
    Matrix2.col(1) = RefFrame2.yVector.normalized();
    Matrix2.col(2) = RefFrame2.zVector.normalized();
  }

  Matrix3d rotMatrix;
  rotMatrix = Matrix1.transpose() * Matrix2;

  getRPY_XYZ(this->eulerAngles,rotMatrix);
  this->eulerAngles *= 180/3.1415;
}

void Joint::getEulerAngles(Vector3d &eulerAngles){
  eulerAngles = this->eulerAngles;
}
