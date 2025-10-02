#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8.h"
#include <vector>


/* Segments Names by order
RFH
RBH
LBH
LFH
C7
IJ_chest
RAC_sho
LAC_sho
RHGT_sho
LHGT_sho
RPSI_back
LPSI_back
RASI
LASI
RLE_elb
RME_elb
LLE_elb
LME_elb
RUS_wrist
RRS_wrist
LUS_wrist
LRS_wrist
RTF_hand
RHM2_hand
RIF_hand
RHL5_hand
RLF_hand
LTF_hand
LHM2_hand
LIF_hand
LHL5_hand
LLF_hand
*/

using namespace std;

class ros_communication
{
private:
  //ros::NodeHandle n;
  ros::Publisher RFH_pos_pub;
  ros::Publisher RBH_pos_pub;
  ros::Publisher LBH_pos_pub;
  ros::Publisher LFH_pos_pub;
  ros::Publisher C7_pos_pub;
  ros::Publisher IJ_chest_pos_pub;
  ros::Publisher RAC_sho_pos_pub;
  ros::Publisher LAC_sho_pos_pub;
  ros::Publisher RHGT_sho_pos_pub;
  ros::Publisher LHGT_sho_pos_pub;
  ros::Publisher RPSI_back_pos_pub;
  ros::Publisher LPSI_back_pos_pub;
  ros::Publisher RASI_pos_pub;
  ros::Publisher LASI_pos_pub;
  ros::Publisher RLE_elb_pos_pub;
  ros::Publisher RME_elb_pos_pub;
  ros::Publisher LLE_elb_pos_pub;
  ros::Publisher LME_elb_pos_pub;
  ros::Publisher RUS_wrist_pos_pub;
  ros::Publisher RRS_wrist_pos_pub;
  ros::Publisher LUS_wrist_pos_pub;
  ros::Publisher LRS_wrist_pos_pub;
  ros::Publisher RTF_hand_pos_pub;
  ros::Publisher RHM2_hand_pos_pub;
  ros::Publisher RIF_hand_pos_pub;
  ros::Publisher RHL5_hand_pos_pub;
  ros::Publisher RLF_hand_pos_pub;
  ros::Publisher LTF_hand_pos_pub;
  ros::Publisher LHM2_hand_pos_pub;
  ros::Publisher LIF_hand_pos_pub;
  ros::Publisher LHL5_hand_pos_pub;
  ros::Publisher LLF_hand_pos_pub;
  //Segments Reference Frames Positions Publishers
  ros::Publisher Right_arm_pos_pub;
  ros::Publisher Right_forearm_pos_pub;
  ros::Publisher Right_wrist_pos_pub;
  ros::Publisher Left_arm_pos_pub;
  ros::Publisher Left_forearm_pos_pub;
  ros::Publisher Left_wrist_pos_pub;
  ros::Publisher Trunk_pos_pub;
  //Segments Reference Frames Orientations Publishers
  ros::Publisher Right_arm_ori_pub;
  ros::Publisher Right_forearm_ori_pub;
  ros::Publisher Right_wrist_ori_pub;
  ros::Publisher Left_arm_ori_pub;
  ros::Publisher Left_forearm_ori_pub;
  ros::Publisher Left_wrist_ori_pub;
  ros::Publisher Trunk_ori_pub;

  vector<ros::Publisher> markersPositionsPubBuffer;
  vector<ros::Publisher> segmentsPositionsPubBuffer;
  vector<ros::Publisher> segmentsOrientationsPubBuffer;

public:
  ros_communication(int argc, char *argv[],ros::NodeHandle n);
  void rosPublishMarkersPositions(vector<vector<float>> positions);
  void rosPublishSegmentsPositions(vector<vector<float>> positions);
  void rosPublishSegmentsOrientations(vector<vector<float>> orientations);
};

#endif // ROS_COMMUNICATION_H
