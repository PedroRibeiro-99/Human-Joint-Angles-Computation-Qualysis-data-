#include "../include/qualysis_data/ros_communication.h"

ros_communication::ros_communication(int argc, char *argv[], ros::NodeHandle n, vector<QStringList> topicsStr){
   /* FOR COPPELIA */
  QStringList markersNames = topicsStr.at(0);
  for(QString markerName : markersNames){
    QString markerTopic = "/vrep/" + markerName + "_pos_topic";
    ros::Publisher markerPub = n.advertise<std_msgs::Float32MultiArray>(markerTopic.toStdString(),1);
    markersPositionsPubBuffer.push_back(markerPub);
  }

  QStringList segmentsNames = topicsStr.at(1);
  for(QString segmentName : segmentsNames){
    QString segmentPosTopic = "/vrep/" + segmentName + "_pos_topic";
    QString segmentOriTopic = "/vrep/" + segmentName + "_ori_topic";
    QString segmentRulaStatusTopic = "/vrep/" + segmentName + "_status_topic";
    ros::Publisher segmentPosPub = n.advertise<std_msgs::Float32MultiArray>(segmentPosTopic.toStdString(),1);
    ros::Publisher segmentOriPub = n.advertise<std_msgs::Float32MultiArray>(segmentOriTopic.toStdString(),1);
    ros::Publisher segmentsRulaStatusPub = n.advertise<std_msgs::UInt8>(segmentRulaStatusTopic.toStdString(),1);
    segmentsPositionsPubBuffer.push_back(segmentPosPub);
    segmentsOrientationsPubBuffer.push_back(segmentOriPub);
    segmentsRulaStatusPubBuffer.push_back(segmentsRulaStatusPub);
  }
}

void ros_communication::rosPublishMarkersPositions(vector<vector<float>> positions){

  std_msgs::Float32MultiArray pos_msg;
  vector<float> pos;
  int n_markers = static_cast<int> (this->markersPositionsPubBuffer.size());

  //ros::Rate r(40);

  for(int marker = 0; marker < n_markers; marker++){
    pos = positions.at(marker);
    pos_msg.data = {pos.at(0),pos.at(1),pos.at(2)}; //pass the x,y,z coordinates to pos_msg
    markersPositionsPubBuffer.at(marker).publish(pos_msg);
  }
  //r.sleep();
}

void ros_communication::rosPublishSegmentsPositions(vector<vector<float>> positions){
  std_msgs::Float32MultiArray pos_msg;
  vector<float> pos;
  int n_segments = static_cast<int> (this->segmentsPositionsPubBuffer.size());

  //ros::Rate r(40);

  for(int segment = 0; segment < n_segments; segment++){
    pos = positions.at(segment);
    pos_msg.data = {pos.at(0),pos.at(1),pos.at(2)}; //pass the x,y,z coordinates to pos_msg
    segmentsPositionsPubBuffer.at(segment).publish(pos_msg);
  }
  //r.sleep();
}

void ros_communication::rosPublishSegmentsOrientations(vector<vector<float>> orientations){
  std_msgs::Float32MultiArray ori_msg;
  vector<float> ori;
  int n_segments = static_cast<int> (this->segmentsOrientationsPubBuffer.size());

  //ros::Rate r(40);

  for(int segment = 0; segment < n_segments; segment++){
    ori = orientations.at(segment);
    ori_msg.data = {ori.at(0),ori.at(1),ori.at(2)}; //pass the x,y,z coordinates to ori_msg
    segmentsOrientationsPubBuffer.at(segment).publish(ori_msg);
  }
  //r.sleep();
}

void ros_communication::rosPublishSegmentsRulaStatus(vector<int> rulaStatusBuffer){
  std_msgs::UInt8 rulaStatus_msg;
  size_t n_segments = this->segmentsRulaStatusPubBuffer.size();
  //ros::Rate r(40);

  for(size_t n_segment = 0; n_segment < n_segments; n_segment++){
    rulaStatus_msg.data = rulaStatusBuffer.at(n_segment);
    segmentsRulaStatusPubBuffer.at(n_segment).publish(rulaStatus_msg);
  }
 // r.sleep();
}

void ros_communication::rosPublish(vector<vector<float>> markersPositions, vector<vector<float>> segmentsPositions, vector<vector<float>> segmentsOrientations){
  this->rosPublishMarkersPositions(markersPositions);
  this->rosPublishSegmentsPositions(segmentsPositions);
  this->rosPublishSegmentsOrientations(segmentsOrientations);
}
