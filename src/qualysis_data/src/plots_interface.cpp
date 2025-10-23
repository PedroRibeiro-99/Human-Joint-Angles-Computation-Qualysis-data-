#include "../include/qualysis_data/plots_interface.h"
#include "ui_plots_interface.h"
#include <iostream>
#include <math.h>

PlotsInterface::PlotsInterface(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::PlotsInterface)
{
  ui->setupUi(this);

  this->plots.push_back(this->ui->R_UpperArm_x);
  this->plots.push_back(this->ui->R_UpperArm_y);
  this->plots.push_back(this->ui->R_UpperArm_z);
  this->plots.push_back(this->ui->R_ForeArm_x);
  this->plots.push_back(this->ui->R_ForeArm_y);
  this->plots.push_back(this->ui->R_ForeArm_z);
  this->plots.push_back(this->ui->R_Wrist_x);
  this->plots.push_back(this->ui->R_Wrist_y);
  this->plots.push_back(this->ui->R_Wrist_z);

  this->plots.push_back(this->ui->L_UpperArm_x);
  this->plots.push_back(this->ui->L_UpperArm_y);
  this->plots.push_back(this->ui->L_UpperArm_z);
  this->plots.push_back(this->ui->L_ForeArm_x);
  this->plots.push_back(this->ui->L_ForeArm_y);
  this->plots.push_back(this->ui->L_ForeArm_z);
  this->plots.push_back(this->ui->L_Wrist_x);
  this->plots.push_back(this->ui->L_Wrist_y);
  this->plots.push_back(this->ui->L_Wrist_z);

  this->plots.push_back(this->ui->Neck_x);
  this->plots.push_back(this->ui->Neck_y);
  this->plots.push_back(this->ui->Neck_z);
  this->plots.push_back(this->ui->Trunk_x);
  this->plots.push_back(this->ui->Trunk_y);
  this->plots.push_back(this->ui->Trunk_z);

  //Plot Features
  for(QCustomPlot *plot : plots){
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    connect(plot, SIGNAL(mouseDoubleClick(QMouseEvent*)), this, SLOT(resetZoom()));
  }
}

PlotsInterface::~PlotsInterface()
{
  delete ui;
}

void PlotsInterface::reset_plots(){

  for(QCustomPlot *plot : this->plots){
    plot->clearGraphs();
    plot->addGraph();
    plot->replot();
  }
}


void PlotsInterface::update_plots(int frame, vector<Joint*> joints){
  Joint *joint;
  Vector3d eulerAngles; //x,y,z;

  for(size_t j = 0 ; j < joints.size(); j++){
    joint = joints.at(j);
    if(joint->getDataStatus()){
      joint->getEulerAngles(eulerAngles);

      this->plots.at(3*j)->graph(0)->addData(frame, eulerAngles.x());  //x component
      this->plots.at(3*j+1)->graph(0)->addData(frame, eulerAngles.y()); //y component
      this->plots.at(3*j+2)->graph(0)->addData(frame, eulerAngles.z()); //z component
    }
    else{
      cout << "failed at frame: " << frame << endl;
      this->plots.at(3*j)->graph(0)->addData(frame,qQNaN());  //x component
      this->plots.at(3*j+1)->graph(0)->addData(frame, qQNaN()); //y component
      this->plots.at(3*j+2)->graph(0)->addData(frame, qQNaN()); //z component
    }
  }
}

void PlotsInterface::draw_plots(){
  for(QCustomPlot *plot : this->plots){
    plot->graph(0)->keyAxis()->setLabel("frame");
    plot->graph(0)->valueAxis()->setLabel("angle");
    plot->graph(0)->valueAxis()->setRange(-180,180);
    plot->graph(0)->keyAxis()->setRange(0,plot->graph(0)->data()->size());
    plot->replot();
  }
}

void PlotsInterface::resetZoom()
{
    QCustomPlot *plot = qobject_cast<QCustomPlot*>(sender());
    if (plot) {
      plot->graph(0)->valueAxis()->setRange(-180,180);
      plot->graph(0)->keyAxis()->setRange(0,plot->graph(0)->data()->size());
      plot->replot();
    }
}

