#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8.h"
#include <vector>
#include <QStringList>


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
  vector<ros::Publisher> markersPositionsPubBuffer;
  vector<ros::Publisher> segmentsPositionsPubBuffer;
  vector<ros::Publisher> segmentsOrientationsPubBuffer;
  vector<ros::Publisher> segmentsRulaStatusPubBuffer;

public:
  ros_communication(int argc, char *argv[],ros::NodeHandle n, vector<QStringList> topicsStr);
  void rosPublishMarkersPositions(vector<vector<float>> positions);
  void rosPublishSegmentsPositions(vector<vector<float>> positions);
  void rosPublishSegmentsOrientations(vector<vector<float>> orientations);
  void rosPublishSegmentsRulaStatus(vector<int> rulaStatus);
  void rosPublish(vector<vector<float>> markersPositions, vector<vector<float>> segmentsPositions, vector<vector<float>> segmentsOrientations);
};

#endif // ROS_COMMUNICATION_H
