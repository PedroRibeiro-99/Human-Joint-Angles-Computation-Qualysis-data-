#include "../include/qualysis_data/segment.h"


void getRPY(Vector3d &eulerAngles, Matrix3d Rot){

  if((Rot(0,0)<1e-10) && (Rot(1,0)<1e-10))
  {// singularity
      eulerAngles.z() = 0; // roll
      eulerAngles.y() = std::atan2(-Rot(2,0),Rot(0,0)); // pitch
      eulerAngles.x() = std::atan2(-Rot(1,2),Rot(1,1)); // yaw
  }
  else
  {
      eulerAngles.z() = std::atan2(Rot(1,0),Rot(0,0)); // roll
      double sp = std::sin(eulerAngles.z()); double cp = std::cos(eulerAngles.z());
      eulerAngles.y() = std::atan2(-Rot(2,0),cp*Rot(0,0)+sp*Rot(1,0)); // pitch
      eulerAngles.x() = std::atan2(sp*Rot(0,2)-cp*Rot(1,2),cp*Rot(1,1)-sp*Rot(0,1)); // yaw
  }
}


void getRPY_XYZ(Vector3d &eulerAngles, Matrix3d R) {
    const double EPS = 1e-8;
    double sy = R(0,2); // sin(pitch)

    if (std::abs(sy) >= 1.0 - EPS) {

        eulerAngles.y() = (sy > 0 ? M_PI/2.0 : -M_PI/2.0); // pitch
        eulerAngles.x() = std::atan2(R(2,1), R(1,1));      // roll + yaw
        eulerAngles.z() = 0.0;                             // yaw
    } else {
        eulerAngles.y() = std::asin(sy);                      // pitch
        eulerAngles.x() = std::atan2(-R(1,2), R(2,2));        // roll
        eulerAngles.z() = std::atan2(-R(0,1), R(0,0));        // yaw
    }
}



Segment::Segment()
{
}
void Segment::setName(string name){
  this->name = name;
}

string Segment::getName(){
  return this->name;
}


void Segment::initSegment(vector<int> markersId, int normDirection, int dirDirection, Vector3d calibrationAngles){
  this->markersId = markersId;
  this->normDirection = normDirection;
  this->dirDirection = dirDirection;
  this->calibrationAngles = calibrationAngles;

  this->calibrationAngles.x() = this->calibrationAngles.x() * 3.1415/180;
  this->calibrationAngles.y() = this->calibrationAngles.y() * 3.1415/180;
  this->calibrationAngles.z() = this->calibrationAngles.z() * 3.1415/180;
}

void resetCoordinateRefFrame(CoordinateRefFrame &referenceFrame){
  referenceFrame.xVector = {0,0,0};
  referenceFrame.yVector = {0,0,0};
  referenceFrame.zVector = {0,0,0};
  referenceFrame.position = {0,0,0};
  referenceFrame.eulerAngles = {0,0,0}; //X-Y-Z
}

void Segment::resetData(){
  resetCoordinateRefFrame(this->baseRefFrame);
  resetCoordinateRefFrame(this->calibratedRefFrame);
}

bool Segment::verifyData(QualysisMoveData &q_obj,int frame){

  //if a necessary marker to define the segment is not available
  for(int markerID : this->markersId){
    if(markerID == -1) {
      this->validData = false;
      return validData;
    }
  }

  vector<Marker> markers;
  Marker marker;
  for(int i = 0; i < this->markersId.size(); i++){
    q_obj.getMarkerData(marker,markersId.at(i),frame);
    markers.push_back(marker);
  }

  this->validData = true;
  for(Marker marker : markers){
    if(marker.coordinates.x == 0 && marker.coordinates.y == 0 && marker.coordinates.z == 0){
      this->validData = false;
      continue;
    }
  }
  return this->validData;
}


bool Segment::getDataStatus(){
  return this->validData;
}


//Compute the normal vector for the base reference frame of the segment
void Segment::computeNormalVector(QualysisMoveData &q_obj, int frame){
  vector<Marker> markers;
  Marker marker;
  for(int i = 0; i < this->markersId.size(); i++){
    q_obj.getMarkerData(marker,markersId.at(i),frame);
    markers.push_back(marker);
  }

  float dir1_x = 0, dir1_y = 0, dir1_z = 0;
  float dir2_x = 0, dir2_y = 0, dir2_z = 0;

  dir1_x = markers.at(1).coordinates.x - markers.at(0).coordinates.x;
  dir1_y = markers.at(1).coordinates.y - markers.at(0).coordinates.y;
  dir1_z = markers.at(1).coordinates.z - markers.at(0).coordinates.z;

  if (markers.size() == 3){
    dir2_x = markers.at(2).coordinates.x - markers.at(0).coordinates.x;
    dir2_y = markers.at(2).coordinates.y - markers.at(0).coordinates.y;
    dir2_z = markers.at(2).coordinates.z - markers.at(0).coordinates.z;
  }

  else if (markers.size() == 4){
    Position m_point;
    m_point.x = (markers.at(3).coordinates.x + markers.at(2).coordinates.x)/2;
    m_point.y = (markers.at(3).coordinates.y + markers.at(2).coordinates.y)/2;
    m_point.z = (markers.at(3).coordinates.z + markers.at(2).coordinates.z)/2;

    dir2_x = m_point.x - markers.at(0).coordinates.x;
    dir2_y = m_point.y - markers.at(0).coordinates.y;
    dir2_z = m_point.z - markers.at(0).coordinates.z;
  }

  Vector3d v1(dir1_x,dir1_y,dir1_z);
  Vector3d v2(dir2_x,dir2_y,dir2_z);

  if(normDirection == 1)
    this->baseRefFrame.xVector = v1.cross(v2);
  else if(normDirection == -1)
    this->baseRefFrame.xVector = v2.cross(v1);
  this->baseRefFrame.xVector.normalize();
}


//Compute the normal vector for the base reference frame of the segment
//z_dir is used to indicate the direction of tail->point of the vector. This is mainly used when the segment considers 3 markers
void Segment::computeDirectorVector(QualysisMoveData &q_obj, int frame){
  vector<Marker> markers;
  Marker marker;
  for(int i = 0; i < this->markersId.size(); i++){
    q_obj.getMarkerData(marker,markersId.at(i),frame);
    markers.push_back(marker);
  }

  float dir_x = 0, dir_y = 0, dir_z = 0;

  if(markers.size() == 2){
    dir_x = markers.at(1).coordinates.x - markers.at(0).coordinates.x;
    dir_y = markers.at(1).coordinates.y - markers.at(0).coordinates.y;
    dir_z = markers.at(1).coordinates.z - markers.at(0).coordinates.z;
  }

  else if (markers.size() == 3){
    Position m_point;
    m_point.x = (markers.at(1).coordinates.x + markers.at(0).coordinates.x)/2;
    m_point.y = (markers.at(1).coordinates.y + markers.at(0).coordinates.y)/2;
    m_point.z = (markers.at(1).coordinates.z + markers.at(0).coordinates.z)/2;

    dir_x = markers.at(2).coordinates.x - m_point.x;
    dir_y = markers.at(2).coordinates.y - m_point.y;
    dir_z = markers.at(2).coordinates.z - m_point.z;
  }

  else if (markers.size() == 4){
    Position m_point1;
    m_point1.x = (markers.at(1).coordinates.x + markers.at(0).coordinates.x)/2;
    m_point1.y = (markers.at(1).coordinates.y + markers.at(0).coordinates.y)/2;
    m_point1.z = (markers.at(1).coordinates.z + markers.at(0).coordinates.z)/2;

    Position m_point2;
    m_point2.x = (markers.at(3).coordinates.x + markers.at(2).coordinates.x)/2;
    m_point2.y = (markers.at(3).coordinates.y + markers.at(2).coordinates.y)/2;
    m_point2.z = (markers.at(3).coordinates.z + markers.at(2).coordinates.z)/2;

    dir_x = m_point2.x - m_point1.x;
    dir_y = m_point2.y - m_point1.y;
    dir_z = m_point2.z - m_point1.z;
  }

  this->baseRefFrame.zVector(0) = dir_x * dirDirection;
  this->baseRefFrame.zVector(1) = dir_y * dirDirection;
  this->baseRefFrame.zVector(2) = dir_z * dirDirection;
  this->baseRefFrame.zVector.normalize();
}

//Compute the y vector for the base reference frame of the segment, using the right handed rule
void Segment::computeYVector(){
  //Right Handed Rule
  this->baseRefFrame.yVector = this->baseRefFrame.zVector.cross(this->baseRefFrame.xVector);
}


void Segment::computePosition(QualysisMoveData &q_obj, int frame){
  vector<Marker> markers;
  Marker marker;
  Vector3d position = {0,0,0};

  for(int i = 0; i < this->markersId.size(); i++){
    q_obj.getMarkerData(marker,markersId.at(i),frame);
    markers.push_back(marker);
  }

  Vector3d baseMiddlePoint((markers.at(0).coordinates.x + markers.at(1).coordinates.x)/2,
                           (markers.at(0).coordinates.y + markers.at(1).coordinates.y)/2,
                           (markers.at(0).coordinates.z + markers.at(1).coordinates.z)/2);

  Vector3d lenghtVector(markers.at(2).coordinates.x - baseMiddlePoint.x(),
                        markers.at(2).coordinates.y - baseMiddlePoint.y(),
                        markers.at(2).coordinates.z - baseMiddlePoint.z());


  position = baseMiddlePoint + this->baseRefFrame.zVector * lenghtVector.norm()/2;
  this->baseRefFrame.position = position;
}


void Segment::computeBaseRefFrameRPY(){
  Matrix3d baseRefFrameMatrix;
  baseRefFrameMatrix.col(0) = baseRefFrame.xVector.normalized();
  baseRefFrameMatrix.col(1) = baseRefFrame.yVector.normalized();
  baseRefFrameMatrix.col(2) = baseRefFrame.zVector.normalized();

  getRPY(this->baseRefFrame.eulerAngles,baseRefFrameMatrix);
  this->baseRefFrame.eulerAngles *=180/3.1415;
}


//Compute Base Reference Frames from the plane formed by the markers
void Segment::computeBaseRefFrame(QualysisMoveData &q_obj, int frame){
  this->computeNormalVector(q_obj,frame);
  this->computeDirectorVector(q_obj,frame);
  this->computeYVector();
  this->computePosition(q_obj,frame);
  this->computeBaseRefFrameRPY();
}




void computeRotationMatrix(Vector3d rotationAngles, Matrix3d &rotationMatrix){
  Matrix3d matrixX, matrixY, matrixZ;
  float x = rotationAngles.x();
  float y = rotationAngles.y();
  float z = rotationAngles.z();

  matrixX << 1, 0, 0,
             0, cos(x), -sin(x),
             0, sin(x), cos(x);

  matrixY << cos(y), 0, sin(y),
             0, 1, 0,
             -sin(y), 0, cos(y);

  matrixZ << cos(z), -sin(z), 0,
             sin(z), cos(z), 0,
             0, 0, 1;

  rotationMatrix = matrixZ * matrixY * matrixX; //rotation matrix related to the original orientation
}


//Compute Calibrated Reference frame by rotating the Base Reference Frame in order to respect the standard reference frames for the neutral posture
void Segment::computeCalibratedRefFrame(){
  Matrix3d baseRefFrameMatrix;
  baseRefFrameMatrix.col(0) = this->baseRefFrame.xVector.normalized();
  baseRefFrameMatrix.col(1) = this->baseRefFrame.yVector.normalized();
  baseRefFrameMatrix.col(2) = this->baseRefFrame.zVector.normalized();

  Matrix3d rotationMatrix;
  computeRotationMatrix(this->calibrationAngles,rotationMatrix);

  Matrix3d CalibratedRefFrameMatrix;
  CalibratedRefFrameMatrix = baseRefFrameMatrix * rotationMatrix;

  this->calibratedRefFrame.xVector = CalibratedRefFrameMatrix.col(0).normalized();
  this->calibratedRefFrame.yVector = CalibratedRefFrameMatrix.col(1).normalized();
  this->calibratedRefFrame.zVector = CalibratedRefFrameMatrix.col(2).normalized();

  getRPY(this->calibratedRefFrame.eulerAngles,CalibratedRefFrameMatrix);
  this->calibratedRefFrame.eulerAngles*=180/3.1415;

  this->calibratedRefFrame.position = this->baseRefFrame.position;
}


void Segment::getBaseRefFrame(CoordinateRefFrame &baseRefFrame){
  baseRefFrame = this->baseRefFrame;
}


void Segment::getCalibratedRefFrame(CoordinateRefFrame &calibratedRefFrame){
  calibratedRefFrame = this->calibratedRefFrame;
}
