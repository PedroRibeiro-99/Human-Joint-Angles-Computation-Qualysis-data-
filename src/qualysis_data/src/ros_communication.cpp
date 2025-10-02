#include "../include/qualysis_data/ros_communication.h"

ros_communication::ros_communication(int argc, char *argv[], ros::NodeHandle n){
   /* FOR COPPELIA */
  RFH_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RFH_pos_topic", 1);
  RBH_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RBH_pos_topic", 1);
  LBH_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LBH_pos_topic", 1);
  LFH_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LFH_pos_topic", 1);
  C7_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/C7_pos_topic", 1);
  IJ_chest_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/IJ_chest_pos_topic", 1);
  RAC_sho_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RAC_sho_pos_topic", 1);
  LAC_sho_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LAC_sho_pos_topic", 1);
  RHGT_sho_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RHGT_sho_pos_topic", 1);
  LHGT_sho_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LHGT_sho_pos_topic", 1);
  RPSI_back_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RPSI_back_pos_topic", 1);
  LPSI_back_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LPSI_back_pos_topic", 1);
  RASI_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RASI_pos_topic", 1);
  LASI_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LASI_pos_topic", 1);
  RLE_elb_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RLE_elb_pos_topic", 1);
  RME_elb_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RME_elb_pos_topic", 1);
  LLE_elb_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LLE_elb_pos_topic", 1);
  LME_elb_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LME_elb_pos_topic", 1);
  RUS_wrist_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RUS_wrist_pos_topic", 1);
  RRS_wrist_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RRS_wrist_pos_topic", 1);
  LUS_wrist_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LUS_wrist_pos_topic", 1);
  LRS_wrist_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LRS_wrist_pos_topic", 1);
  RTF_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RTF_hand_pos_topic", 1);
  RHM2_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RHM2_hand_pos_topic", 1);
  RIF_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RIF_hand_pos_topic", 1);
  RHL5_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RHL5_hand_pos_topic", 1);
  RLF_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/RLF_hand_pos_topic", 1);
  LTF_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LTF_hand_pos_topic", 1);
  LHM2_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LHM2_hand_pos_topic", 1);
  LIF_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LIF_hand_pos_topic", 1);
  LHL5_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LHL5_hand_pos_topic", 1);
  LLF_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/LLF_hand_pos_topic", 1);
  markersPositionsPubBuffer.push_back(RFH_pos_pub);
  markersPositionsPubBuffer.push_back(RBH_pos_pub);
  markersPositionsPubBuffer.push_back(LBH_pos_pub);
  markersPositionsPubBuffer.push_back(LFH_pos_pub);
  markersPositionsPubBuffer.push_back(C7_pos_pub);
  markersPositionsPubBuffer.push_back(IJ_chest_pos_pub);
  markersPositionsPubBuffer.push_back(RAC_sho_pos_pub);
  markersPositionsPubBuffer.push_back(LAC_sho_pos_pub);
  markersPositionsPubBuffer.push_back(RHGT_sho_pos_pub);
  markersPositionsPubBuffer.push_back(LHGT_sho_pos_pub);
  markersPositionsPubBuffer.push_back(RPSI_back_pos_pub);
  markersPositionsPubBuffer.push_back(LPSI_back_pos_pub);
  markersPositionsPubBuffer.push_back(RASI_pos_pub);
  markersPositionsPubBuffer.push_back(LASI_pos_pub);
  markersPositionsPubBuffer.push_back(RLE_elb_pos_pub);
  markersPositionsPubBuffer.push_back(RME_elb_pos_pub);
  markersPositionsPubBuffer.push_back(LLE_elb_pos_pub);
  markersPositionsPubBuffer.push_back(LME_elb_pos_pub);
  markersPositionsPubBuffer.push_back(RUS_wrist_pos_pub);
  markersPositionsPubBuffer.push_back(RRS_wrist_pos_pub);
  markersPositionsPubBuffer.push_back(LUS_wrist_pos_pub);
  markersPositionsPubBuffer.push_back(LRS_wrist_pos_pub);
  markersPositionsPubBuffer.push_back(RTF_hand_pos_pub);
  markersPositionsPubBuffer.push_back(RHM2_hand_pos_pub);
  markersPositionsPubBuffer.push_back(RIF_hand_pos_pub);
  markersPositionsPubBuffer.push_back(RHL5_hand_pos_pub);
  markersPositionsPubBuffer.push_back(RLF_hand_pos_pub);
  markersPositionsPubBuffer.push_back(LTF_hand_pos_pub);
  markersPositionsPubBuffer.push_back(LHM2_hand_pos_pub);
  markersPositionsPubBuffer.push_back(LIF_hand_pos_pub);
  markersPositionsPubBuffer.push_back(LHL5_hand_pos_pub);
  markersPositionsPubBuffer.push_back(LLF_hand_pos_pub);

  //Segments Reference Frames Positions Publishers
  Right_arm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Right_arm_pos_topic", 1);
  Right_forearm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Right_forearm_pos_topic", 1);
  Right_wrist_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Right_wrist_pos_topic", 1);
  Left_arm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Left_arm_pos_topic", 1);
  Left_forearm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Left_forearm_pos_topic", 1);
  Left_wrist_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Left_wrist_pos_topic", 1);
  Trunk_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Trunk_pos_topic", 1);
  segmentsPositionsPubBuffer.push_back(Right_arm_pos_pub);
  segmentsPositionsPubBuffer.push_back(Right_forearm_pos_pub);
  segmentsPositionsPubBuffer.push_back(Right_wrist_pos_pub);
  segmentsPositionsPubBuffer.push_back(Left_arm_pos_pub);
  segmentsPositionsPubBuffer.push_back(Left_forearm_pos_pub);
  segmentsPositionsPubBuffer.push_back(Left_wrist_pos_pub);
  segmentsPositionsPubBuffer.push_back(Trunk_pos_pub);

  //Segments Reference Frames Orientations Publishers
  Right_arm_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Right_arm_ori_topic", 1);
  Right_forearm_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Right_forearm_ori_topic", 1);
  Right_wrist_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Right_wrist_ori_topic", 1);
  Left_arm_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Left_arm_ori_topic", 1);
  Left_forearm_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Left_forearm_ori_topic", 1);
  Left_wrist_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Left_wrist_ori_topic", 1);
  Trunk_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/Trunk_ori_topic", 1);
  segmentsOrientationsPubBuffer.push_back(Right_arm_ori_pub);
  segmentsOrientationsPubBuffer.push_back(Right_forearm_ori_pub);
  segmentsOrientationsPubBuffer.push_back(Right_wrist_ori_pub);
  segmentsOrientationsPubBuffer.push_back(Left_arm_ori_pub);
  segmentsOrientationsPubBuffer.push_back(Left_forearm_ori_pub);
  segmentsOrientationsPubBuffer.push_back(Left_wrist_ori_pub);
  segmentsOrientationsPubBuffer.push_back(Trunk_ori_pub);
}

void ros_communication::rosPublishMarkersPositions(vector<vector<float>> positions){

  std_msgs::Float32MultiArray pos_msg;
  vector<float> pos;
  int n_markers = static_cast<int> (this->markersPositionsPubBuffer.size());

  ros::Rate r(40);

  for(int marker = 0; marker < n_markers; marker++){
    pos = positions.at(marker);
    pos_msg.data = {pos.at(0),pos.at(1),pos.at(2)}; //pass the x,y,z coordinates to pos_msg
    markersPositionsPubBuffer.at(marker).publish(pos_msg);
  }
  r.sleep();
}

void ros_communication::rosPublishSegmentsPositions(vector<vector<float>> positions){
  std_msgs::Float32MultiArray pos_msg;
  vector<float> pos;
  int n_segments = static_cast<int> (this->segmentsPositionsPubBuffer.size());

  ros::Rate r(40);

  for(int segment = 0; segment < n_segments; segment++){
    pos = positions.at(segment);
    pos_msg.data = {pos.at(0),pos.at(1),pos.at(2)}; //pass the x,y,z coordinates to pos_msg
    segmentsPositionsPubBuffer.at(segment).publish(pos_msg);
  }
  r.sleep();
}

void ros_communication::rosPublishSegmentsOrientations(vector<vector<float>> orientations){
  std_msgs::Float32MultiArray ori_msg;
  vector<float> ori;
  int n_segments = static_cast<int> (this->segmentsOrientationsPubBuffer.size());

  ros::Rate r(40);

  for(int segment = 0; segment < n_segments; segment++){
    ori = orientations.at(segment);
    ori_msg.data = {ori.at(0),ori.at(1),ori.at(2)}; //pass the x,y,z coordinates to pos_msg
    segmentsOrientationsPubBuffer.at(segment).publish(ori_msg);
  }
  r.sleep();
}
