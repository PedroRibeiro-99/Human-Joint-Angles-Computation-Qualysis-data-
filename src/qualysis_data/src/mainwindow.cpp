#include "../include/qualysis_data/mainwindow.h"
#include "../include/qualysis_data/config.hpp"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QFile>
#include <QThread>

#if SIMULATION == 0
#include <vrep_common/simRosLoadScene.h>
#include <vrep_common/simRosCloseScene.h>
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosPauseSimulation.h>
#include <QProcess>
#endif


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  sR_arm.setName("Right_arm");
  sR_forearm.setName("Right_forearm");
  sR_wrist.setName("Right_wrist");
  s_trunk.setName("Trunk");
  sL_arm.setName("Left_arm");
  sL_forearm.setName("Left_forearm");
  sL_wrist.setName("Left_wrist");
  this->segments_buffer = {&sR_arm, &sR_forearm, &sR_wrist, &sL_arm, &sL_forearm, &sL_wrist, &s_trunk};


  this->jR_arm.initJoint("RIGHT ARM",&s_trunk, &sR_arm);
  this->jR_forearm.initJoint("RIGHT FOREARM",&sR_arm, &sR_forearm);
  this->jR_wrist.initJoint("RIGHT WRIST",&sR_forearm, &sR_wrist);
  this->jL_arm.initJoint("LEFT ARM",&s_trunk, &sL_arm);
  this->jL_forearm.initJoint("LEFT FOREARM",&sL_arm, &sL_forearm);
  this->jL_wrist.initJoint("LEFT WRIST",&sL_forearm, &sL_wrist);
  this->joints_buffer = {&jR_arm, &jR_forearm, &jR_wrist, &jL_arm, &jL_forearm, &jL_wrist};

#if SIMULATION == 0
  ros::init(argc, argv, "QualysisROSComunication");
  if(this->initSimulationEnvironment() == -1) exit(-1);
#endif
}

MainWindow::~MainWindow()
{
  delete ui;
}


int appendJointsData(QFile &file, vector<Joint*> joints, int frame){
  QTextStream in(&file);

  Vector3d eulerAngles = {0,0,0};
  Joint *joint_obj;

  in << "FRAME: " << frame << endl << endl;
  for(size_t joint = 0; joint < joints.size(); joint++){
    joint_obj = joints.at(joint);
    in << QString::fromStdString(joint_obj->getName()) << endl;
    if(joint_obj->getDataStatus()){
      joint_obj->getEulerAngles(eulerAngles);
      in << "Rotation in X " << eulerAngles.x() << endl;
      in << "Rotation in Y " << eulerAngles.y() << endl;
      in << "Rotation in Z " << eulerAngles.z() << endl << endl;
    }
    else {
      in << "Rotation in X: NULL DATA" << endl;
      in << "Rotation in Y: NULL DATA" << endl;
      in << "Rotation in Z: NULL DATA" << endl << endl;
    }
  }

  in << "-----------------------------------------------------" << endl;
}



void MainWindow::on_pushButton_loadfile_clicked(){
  this->plot_interface.reset_plots();
  this->q_obj.resetData();

  QString file_name = QFileDialog::getOpenFileName(this,"",FILES_PATH);
  char *file_path = file_name.toLocal8Bit().data();

  if(this->q_obj.load_data_file(file_path) == 1){
    //SEGMENTS
    this->sR_arm.initSegment(this->q_obj.getMarkersIDs(vector<string>{"RLE_elb","RME_elb","RAC_sho"}),1,1,{0,0,-127});
    this->sR_forearm.initSegment(this->q_obj.getMarkersIDs(vector<string>{"RUS_wrist","RRS_wrist","RLE_elb"}),-1,1,{0,0,-191});
    this->sR_wrist.initSegment(this->q_obj.getMarkersIDs(vector<string>{"RHM2_hand","RHL5_hand","RUS_wrist","RRS_wrist"}),1,1,{0,0,-187});
    this->sL_arm.initSegment(this->q_obj.getMarkersIDs(vector<string>{"LLE_elb","LME_elb","LAC_sho"}),-1,1,{0,0,-45});
    this->sL_forearm.initSegment(this->q_obj.getMarkersIDs(vector<string>{"LUS_wrist","LRS_wrist","LLE_elb"}),1,1,{0,0,15});
    this->sL_wrist.initSegment(this->q_obj.getMarkersIDs(vector<string>{"LHM2_hand","LHL5_hand","LUS_wrist","LRS_wrist"}),-1,1,{0,0,10});
    this->s_trunk.initSegment(this->q_obj.getMarkersIDs(vector<string>{"RPSI_back","LPSI_back","C7"}),1,1,{0,0,-90});

    QString write_file = file_name;
    write_file.replace("tsv_files", "reports");
    write_file.replace(".tsv",".txt");

    QFile file(write_file);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "Error when open the file:" << file.errorString();
    }
    file.close();

    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        qDebug() << "Error when open the file:" << file.errorString();
    }

    Vector3d euler_angles;
    for(int frame = 0; frame < this->q_obj.get_n_frames(); frame++){
      for(Segment *segment : this->segments_buffer){
        if(segment->verifyData(this->q_obj,frame)){
          segment->computeBaseRefFrame(this->q_obj,frame);
          segment->computeCalibratedRefFrame();
        }
        else {
          cout << "failed at frame " << frame << " for " << segment->getName() << endl;
          segment->resetData();
        }
      }
      for(Joint *joint : this->joints_buffer)
        if(joint->verifyData())
          joint->computeEulerAngles();
        else {
          joint->resetData();
        }

      //Save data in the .txt report
      appendJointsData(file,this->joints_buffer,frame);

      //Update plots data
      this->plot_interface.update_plots(frame,this->joints_buffer);

      cout << frame << endl;
    }
    file.close();
  }
  ui->loadFile_label->setText(file_name);
}


void MainWindow::on_pushButton_plots_clicked(){
    this->plot_interface.draw_plots();
    this->plot_interface.show();
}


int MainWindow::initSimulationEnvironment(){

  if(!ros::master::check())
      ros::start();

  bool online = false;

  ros::V_string nodes;
  ros::master::getNodes(nodes);
  for(string node_name : nodes){
    if(node_name == "/vrep")
        online = true;
  }

  ros::ServiceClient rosClient;
  ros::NodeHandle n;

  if(!online)
    return -1;
  else{
    // pause simulations
    rosClient = n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvstop;
    rosClient.call(srvstop);

    // close the old scene
    rosClient = n.serviceClient<vrep_common::simRosCloseScene>("/vrep/simRosCloseScene");
    vrep_common::simRosCloseScene srvc;
    rosClient.call(srvc);
  }
  // load the new scene
  rosClient = n.serviceClient<vrep_common::simRosLoadScene>("/vrep/simRosLoadScene");
  vrep_common::simRosLoadScene srv;
  srv.request.fileName = COPPELIA_SCENARIO_PATH;
  rosClient.call(srv);

  rosClient = n.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
  vrep_common::simRosStartSimulation srvstart;
  rosClient.call(srvstart);
}


void MainWindow::on_pushButton_execMovement_clicked(){

  //Get segments names
  QStringList segmentsNames;
  for(Segment *segment : this->segments_buffer){
    segmentsNames.push_back(QString::fromStdString(segment->getName()));
  }
  vector<QStringList> topicsStr = {this->q_obj.getMarkersNames(),segmentsNames};

  ros::NodeHandle n;
  ros_communication ros_handler(argc,argv,n,topicsStr);

  vector<Marker> markers_data;
  vector<vector<float>> markersPositions;
  vector<vector<float>> segmentsPositions;
  vector<vector<float>> segmentsOrientations;

  CoordinateRefFrame referenceFrame;

  for(int frame = 0; frame < q_obj.get_n_frames(); frame++){
    markersPositions.clear();
    segmentsPositions.clear();
    segmentsOrientations.clear();

    q_obj.get_frame_data(markers_data,frame);

    for(Segment *segment : this->segments_buffer){
      if(segment->verifyData(this->q_obj,frame)){
        segment->computeBaseRefFrame(this->q_obj,frame);
        segment->computeCalibratedRefFrame();
      }
      else{
        segment->resetData();
      }

      segment->getCalibratedRefFrame(referenceFrame);

      segmentsPositions.push_back({static_cast<float>(referenceFrame.position.x()),
                                   static_cast<float>(referenceFrame.position.y()),
                                   static_cast<float>(referenceFrame.position.z())});
      segmentsOrientations.push_back({static_cast<float>(referenceFrame.eulerAngles.x()),
                                      static_cast<float>(referenceFrame.eulerAngles.y()),
                                      static_cast<float>(referenceFrame.eulerAngles.z())});
    }

    for(Marker marker_data : markers_data)
      markersPositions.push_back({marker_data.coordinates.x, marker_data.coordinates.y, marker_data.coordinates.z});

    ros_handler.rosPublishMarkersPositions(markersPositions);
    ros_handler.rosPublishSegmentsPositions(segmentsPositions);
    ros_handler.rosPublishSegmentsOrientations(segmentsOrientations);
  }

}

void MainWindow::on_pushButton_execFrame_clicked()
{

  //Get segments names
  QStringList segmentsNames;
  for(Segment *segment : this->segments_buffer){
    segmentsNames.push_back(QString::fromStdString(segment->getName()));
  }
  vector<QStringList> topicsStr = {this->q_obj.getMarkersNames(),segmentsNames};

  ros::NodeHandle n;
  ros_communication ros_handler(argc,argv,n,topicsStr);

  vector<Marker> markers_data;
  vector<vector<float>> markersPositions;
  vector<vector<float>> segmentsPositions;
  vector<vector<float>> segmentsOrientations;

  CoordinateRefFrame referenceFrame;

  int frame = ui->lineEdit_frame->text().toInt();
  q_obj.get_frame_data(markers_data,frame);

  //GET SEGMENTS DATA
  for(Segment *segment : this->segments_buffer){
    if(segment->verifyData(this->q_obj,frame)){
      segment->computeBaseRefFrame(this->q_obj,frame);
      segment->computeCalibratedRefFrame();
    }
    else{
      segment->resetData();
    }

    segment->getCalibratedRefFrame(referenceFrame);

    cout << "----------------------------------------------------------------------" << endl;
    cout << "Segment: " << segment->getName() << endl;
    cout << "Director vector: (" << referenceFrame.zVector.x() << "," << referenceFrame.zVector.y() << "," << referenceFrame.zVector.z() << ")" << endl;
    cout << "Normal vector: (" << referenceFrame.xVector.x() << "," << referenceFrame.xVector.y() << "," << referenceFrame.xVector.z() << ")" << endl;
    cout << "Y vector: (" << referenceFrame.yVector.x() << "," << referenceFrame.yVector.y() << "," << referenceFrame.yVector.z() << ")" << endl;
    cout << "Euler Angles (Z,Y,X)" << endl;
    cout << "Z : " << referenceFrame.eulerAngles.z() << endl;
    cout << "Y : " << referenceFrame.eulerAngles.y() << endl;
    cout << "X : " << referenceFrame.eulerAngles.x() << endl;

    segmentsPositions.push_back({static_cast<float>(referenceFrame.position.x()),
                                 static_cast<float>(referenceFrame.position.y()),
                                 static_cast<float>(referenceFrame.position.z())});
    segmentsOrientations.push_back({static_cast<float>(referenceFrame.eulerAngles.x()),
                                    static_cast<float>(referenceFrame.eulerAngles.y()),
                                    static_cast<float>(referenceFrame.eulerAngles.z())});
  }

  for(Marker marker_data : markers_data)
    markersPositions.push_back({marker_data.coordinates.x, marker_data.coordinates.y, marker_data.coordinates.z});

  sleep(1);
  ros_handler.rosPublishMarkersPositions(markersPositions);
  ros_handler.rosPublishSegmentsPositions(segmentsPositions);
  ros_handler.rosPublishSegmentsOrientations(segmentsOrientations);

}
