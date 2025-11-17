#include "../include/qualysis_data/mainwindow.h"
#include "../include/qualysis_data/config.hpp"

#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QFile>
#include <QThread>
#include <thread>
#include <chrono>

#if SIMULATION == 1
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

  sR_arm.setName("RightArm");
  sR_forearm.setName("RightForearm");
  sR_wrist.setName("RightWrist");
  sL_arm.setName("LeftArm");
  sL_forearm.setName("LeftForearm");
  sL_wrist.setName("LeftWrist");
  s_neck.setName("Neck");
  s_trunk.setName("Trunk");
  this->segments_buffer = {&sR_arm, &sR_forearm, &sR_wrist, &sL_arm, &sL_forearm, &sL_wrist, &s_neck, &s_trunk};


  this->jR_arm.initJoint("RIGHT ARM",{&s_trunk, &sR_arm});
  this->jR_forearm.initJoint("RIGHT FOREARM",{&sR_arm, &sR_forearm});
  this->jR_wrist.initJoint("RIGHT WRIST",{&sR_forearm, &sR_wrist});
  this->jL_arm.initJoint("LEFT ARM",{&s_trunk, &sL_arm});
  this->jL_forearm.initJoint("LEFT FOREARM",{&sL_arm, &sL_forearm});
  this->jL_wrist.initJoint("LEFT WRIST",{&sL_forearm, &sL_wrist});
  this->j_neck.initJoint("NECK", {&s_trunk, &s_neck});
  this->j_trunk.initJoint("TRUNK",{&s_trunk});
  this->joints_buffer = {&jR_arm, &jR_forearm, &jR_wrist, &jL_arm, &jL_forearm, &jL_wrist, &j_neck, &j_trunk};

#if SIMULATION == 1
  ros::init(argc, argv, "QualysisROSComunication");
  if(!this->initSimulationEnvironment()) exit(-1);
  //r.reset();
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

#if RULA_ASSESSMENT == 1

void updateRulaDataBuffer(QString side, Rula &rulaObj, QStringList &textStreamBuffer){

  textStreamBuffer.push_back(side.toUpper() + " ARM RULA ASSESSMENT");
  textStreamBuffer.push_back(side + " Upper Arm: " + QString::number(rulaObj.getUpperArmScore()));
  textStreamBuffer.push_back(side + " Lower Arm: " + QString::number(rulaObj.getLowerArmScore()));
  textStreamBuffer.push_back(side + " Wrist: " + QString::number(rulaObj.getWristScore()));
  textStreamBuffer.push_back("Neck: " + QString::number(rulaObj.getNeckScore()));
  textStreamBuffer.push_back("Trunk: " + QString::number(rulaObj.getTrunkScore()));
  textStreamBuffer.push_back("Group A: " + QString::number(rulaObj.getGroupA_Score()));
  textStreamBuffer.push_back("Group B: " + QString::number(rulaObj.getGroupB_Score()));
  textStreamBuffer.push_back(side.toUpper()+ " TOTAL SCORE: " + QString::number(rulaObj.getTotalScore()));
  textStreamBuffer.push_back(" ");
}


void appendRULA_Data(QFile &file, QStringList &textStreamBuffer, int frame){
  QTextStream in(&file);
  in << "FRAME: " << frame << endl << endl;
  for(QString line : textStreamBuffer)
    in << line << endl;
  in << "-----------------------------------------------------" << endl;
}

#endif


void MainWindow::on_pushButton_loadfile_clicked(){
  this->plot_interface.reset_plots();
  this->q_obj.resetData();

  QString file_name = QFileDialog::getOpenFileName(this,"",FILES_PATH);
  char *file_path = file_name.toLocal8Bit().data();

  if(this->q_obj.loadDataFile(file_path) == 1){
    //SEGMENTS
    this->sR_arm.initSegment(this->q_obj.getMarkersIDs(vector<string>{"RLE_elb","RME_elb","RAC_sho"}),1,1,{0,0,-127});
    this->sR_forearm.initSegment(this->q_obj.getMarkersIDs(vector<string>{"RUS_wrist","RRS_wrist","RLE_elb"}),-1,1,{0,0,-191});
    this->sR_wrist.initSegment(this->q_obj.getMarkersIDs(vector<string>{"RHM2_hand","RHL5_hand","RUS_wrist","RRS_wrist"}),1,1,{0,0,-187});
    this->sL_arm.initSegment(this->q_obj.getMarkersIDs(vector<string>{"LLE_elb","LME_elb","LAC_sho"}),-1,1,{0,0,-45});
    this->sL_forearm.initSegment(this->q_obj.getMarkersIDs(vector<string>{"LUS_wrist","LRS_wrist","LLE_elb"}),1,1,{0,0,15});
    this->sL_wrist.initSegment(this->q_obj.getMarkersIDs(vector<string>{"LHM2_hand","LHL5_hand","LUS_wrist","LRS_wrist"}),-1,1,{0,0,10});
    this->s_neck.initSegment(this->q_obj.getMarkersIDs(vector<string>{"LBH","RBH","C7"}),1,-1,{0,0,-90});
    this->s_trunk.initSegment(this->q_obj.getMarkersIDs(vector<string>{"RPSI_back","LPSI_back","C7"}),1,1,{0,0,-90});

    QString jointsReportFileStr = file_name;
    jointsReportFileStr.replace("tsv_files", "reports");
    jointsReportFileStr.replace(".tsv",".txt");

    QFile jointsReportFile(jointsReportFileStr);
    if (!jointsReportFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "Error when open the file:" << jointsReportFile.errorString();
    }
    jointsReportFile.close();

    if (!jointsReportFile.open(QIODevice::Append | QIODevice::Text)) {
        qDebug() << "Error when open the file:" << jointsReportFile.errorString();
    }

#if RULA_ASSESSMENT == 1
    QString rulaReportFileStr = file_name;
    rulaReportFileStr.replace("tsv_files", "rula_reports");
    rulaReportFileStr.replace(".tsv",".txt");

    QFile rulaReportFile(rulaReportFileStr);
    if (!rulaReportFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "Error when open the file:" << rulaReportFile.errorString();
    }
    rulaReportFile.close();

    if (!rulaReportFile.open(QIODevice::Append | QIODevice::Text)) {
        qDebug() << "Error when open the file:" << rulaReportFile.errorString();
    }
#endif

    Vector3d euler_angles;
    for(int frame = 0; frame < this->q_obj.getNumberOfFrames(); frame++){
      for(Segment *segment : this->segments_buffer)
        this->computeSegmentsRefFrames(*segment,frame);

      this->computeJointsAngles();
      //Save data in the .txt report
      appendJointsData(jointsReportFile,this->joints_buffer,frame);

#if RULA_ASSESSMENT == 1
      Rula rRULA,lRULA;
      vector<int> rulaStatusBuffer = {0,0,0,0,0,0,0,0};
      this->computeRulaEvaluation(rRULA, lRULA, rulaStatusBuffer);

      QStringList textStreamBuffer;
      textStreamBuffer.clear();
      if(rightArmAssessmentAvailable) updateRulaDataBuffer("Right",rRULA,textStreamBuffer);
      if(leftArmAssessmentAvailable) updateRulaDataBuffer("Left",lRULA,textStreamBuffer);
      appendRULA_Data(rulaReportFile,textStreamBuffer,frame);
#endif

      //Update plots data
      this->plot_interface.update_plots(frame,this->joints_buffer);

      cout << frame << endl;
    }

    jointsReportFile.close();

#if RULA_ASSESSMENT == 1
    rulaReportFile.close();
#endif

  }
  ui->loadFile_label->setText(file_name);
}


void MainWindow::on_pushButton_plots_clicked(){
    this->plot_interface.draw_plots();
    this->plot_interface.show();
}


bool MainWindow::initSimulationEnvironment(){

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
    return false;
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

  return true;
}


void MainWindow::on_pushButton_execMovement_clicked(){

  // Criar a thread nativa
  std::thread([this]() {

    //Get segments names
    QStringList segmentsNames;
    for(Segment *segment : this->segments_buffer){
      segmentsNames.push_back(QString::fromStdString(segment->getName()));
    }
    vector<QStringList> topicsStr = {this->q_obj.getMarkersNames(),segmentsNames};

    ros::NodeHandle n;
    ros_communication ros_handler(argc,argv,n,topicsStr);
    usleep(500000);

    for(int frame = 0; frame < q_obj.getNumberOfFrames(); frame++)
        this->execFrameData(frame, ros_handler);

  }).detach(); // detach para rodar em background e não bloquear
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


  int frame = ui->lineEdit_frame->text().toInt();

  usleep(500000);
  this->execFrameData(frame,ros_handler);
}

void MainWindow::execFrameData(int frame, ros_communication &ros_handler){

  //ros::Rate r(this->q_obj.getFrequency());
  ros::Rate r(50);

  vector<Marker> markers_data;
  vector<vector<float>> markersPositions;
  vector<vector<float>> segmentsPositions;
  vector<vector<float>> segmentsOrientations;
  CoordinateRefFrame referenceFrame;

  q_obj.getFrameData(markers_data,frame);

  for(Segment *segment : this->segments_buffer){
    this->computeSegmentsRefFrames(*segment,frame);
    segment->getCalibratedRefFrame(referenceFrame);
    segmentsPositions.push_back({static_cast<float>(referenceFrame.position.x()),
                                 static_cast<float>(referenceFrame.position.y()),
                                 static_cast<float>(referenceFrame.position.z())});
    segmentsOrientations.push_back({static_cast<float>(referenceFrame.eulerAngles.x()),
                                    static_cast<float>(referenceFrame.eulerAngles.y()),
                                    static_cast<float>(referenceFrame.eulerAngles.z())});
  }


  this->computeJointsAngles();

#if RULA_ASSESSMENT == 1
  Rula rRULA,lRULA;
  vector<int> rulaStatusBuffer = {0,0,0,0,0,0,0,0};
  this->computeRulaEvaluation(rRULA, lRULA, rulaStatusBuffer);
  ros_handler.rosPublishSegmentsRulaStatus(rulaStatusBuffer);
#endif

    for(Marker marker_data : markers_data)
      markersPositions.push_back({marker_data.coordinates.x, marker_data.coordinates.y, marker_data.coordinates.z});

    ros_handler.rosPublishMarkersPositions(markersPositions);
    ros_handler.rosPublishSegmentsPositions(segmentsPositions);
    ros_handler.rosPublishSegmentsOrientations(segmentsOrientations);

    r.sleep();
}


void MainWindow::computeSegmentsRefFrames(Segment &segment, int frame){
    if(segment.verifyData(this->q_obj,frame)){
      segment.computeBaseRefFrame(this->q_obj,frame);
      segment.computeCalibratedRefFrame();
    }
    else {
      cout << "failed at frame " << frame << " for " << segment.getName() << endl;
      segment.resetData();
    }
}

void MainWindow::computeJointsAngles(){
  size_t numberOfAvailableJoints = 0;
  for(Joint *joint : this->joints_buffer){
    if(joint->verifyData()){
      joint->computeEulerAngles();
      numberOfAvailableJoints++;
    }
    else {
      joint->resetData();
    }
  }
}

void MainWindow::computeRulaEvaluation(Rula &rRULA, Rula &lRULA, vector<int> &rulaStatusBuffer){
  vector<Joint> rJoints = {*joints_buffer.at(0),*joints_buffer.at(1),*joints_buffer.at(2),*joints_buffer.at(6),*joints_buffer.at(7)};
  vector<Joint> lJoints = {*joints_buffer.at(3),*joints_buffer.at(4),*joints_buffer.at(5),*joints_buffer.at(6),*joints_buffer.at(7)};

  rightArmAssessmentAvailable = true;
  for(Joint joint : rJoints){
    if(!joint.verifyData()){
      rightArmAssessmentAvailable = false;
      break;
    }
  }
  leftArmAssessmentAvailable = true;
  for(Joint joint : lJoints){
    if(!joint.verifyData()){
      leftArmAssessmentAvailable = false;
      break;
    }
  }

  if(rightArmAssessmentAvailable){
    RULA_SegmentsVariables rRulaVariables;
    setRULA_Variables(rJoints, rRulaVariables);
    rRULA.setRULA_Variables(rRulaVariables);
    rRULA.executeRulaEvalutation();
    rulaStatusBuffer.at(0) = rRULA.getUpperArmScoreStatus();
    rulaStatusBuffer.at(1) = rRULA.getLowerArmScoreStatus();
    rulaStatusBuffer.at(2) = rRULA.getWristScoreStatus();
    rulaStatusBuffer.at(6) = rRULA.getNeckScoreStatus();
    rulaStatusBuffer.at(7) = rRULA.getTrunkScoreStatus();

  }
  if(leftArmAssessmentAvailable){
    RULA_SegmentsVariables lRulaVariables;
    setRULA_Variables(lJoints,lRulaVariables);
    lRULA.setRULA_Variables(lRulaVariables);
    lRULA.executeRulaEvalutation();
    rulaStatusBuffer.at(3) = lRULA.getUpperArmScoreStatus();
    rulaStatusBuffer.at(4) = lRULA.getLowerArmScoreStatus();
    rulaStatusBuffer.at(5) = lRULA.getWristScoreStatus();
    rulaStatusBuffer.at(6) = lRULA.getNeckScoreStatus();
    rulaStatusBuffer.at(7) = lRULA.getTrunkScoreStatus();
  }
}
