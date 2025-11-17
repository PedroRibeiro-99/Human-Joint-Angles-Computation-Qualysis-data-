#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "segment.h"
#include "joint.h"
#include "plots_interface.h"
#include "ros_communication.h"
#include "rula.h"
#include <iostream>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
  void computeSegmentsRefFrames(Segment &segment, int frame);
  void computeJointsAngles();
  void computeRulaEvaluation(Rula &rRULA, Rula &lRULA, vector<int> &rulaStatusBuffer);
  void execFrameData(int frame, ros_communication &ros_handler);
  ~MainWindow();

private Q_SLOTS:
  void on_pushButton_loadfile_clicked();
  void on_pushButton_plots_clicked();
  bool initSimulationEnvironment();
  void on_pushButton_execMovement_clicked();
  void on_pushButton_execFrame_clicked();

private:
  int argc; char** argv;
  Ui::MainWindow *ui;
  QualysisMoveData q_obj;
  Segment sR_arm, sR_forearm, sR_wrist, sL_arm, sL_forearm, sL_wrist, s_neck, s_trunk;
  vector<Segment*> segments_buffer;
  Joint jR_arm, jR_forearm, jR_wrist,jL_arm, jL_forearm, jL_wrist, j_neck, j_trunk;
  vector<Joint*> joints_buffer;
  bool rightArmAssessmentAvailable;
  bool leftArmAssessmentAvailable;
  PlotsInterface plot_interface;
};

#endif // MAINWINDOW_H
