#include <QCoreApplication>
#include <QApplication>
#include "../include/qualysis_data/joint.h"
#include "../include/qualysis_data/ros_communication.h"
#include "../include/qualysis_data/mainwindow.h"
#include <iostream>

#define FILE_PATH "src/qualysis_data/files/test_1.tsv" //results session 15

using namespace std;


int save_joints_data(QString write_file, vector<vector<double>> flexions, int frames){

  QFile file(write_file);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      qDebug() << "Erro ao abrir o ficheiro:" << file.errorString();
      return -1;
  }

  QTextStream in(&file);

  for(int frame = 0; frame < frames; frame++){
    vector<double> flexion = flexions.at(frame);
    in << "FRAME: " << frame << endl << endl;

    in << "Right Arm" << endl;
    in << "Flexion: " << flexion.at(0) << endl << endl;

    in << "Right Forearm "<< endl;
    in << "Flexion: " << flexion.at(1) << endl;

    in << "Right Wrist "<< endl;
    in << "Flexion: " << flexion.at(2) << endl;
    in << "-----------------------------------------------------" << endl;
  }
}




/*int append_joints_data(QFile &file, vector<Joint> joints, int frame){
  QTextStream in(&file);

  vector<double> flexions_components = {0,0,0};
  double flexion = 0;
  Joint joint_obj;

  in << "FRAME: " << frame << endl << endl;
  //for(int joint = 0; joint < joints.size(); joint++){
  for(int joint = 0; joint < 1; joint++){
    joint_obj = joints.at(joint);
    joint_obj.get_flexion_components(flexions_components);
    flexion = joint_obj.get_flexion();

    in << QString::fromStdString(joint_obj.get_name()) << endl;
    in << "Flexion: " << flexion << endl;
    in << "Flexion XY: " << flexions_components.at(0) << endl;
    in << "Flexion XZ: " << flexions_components.at(1) << endl;
    in << "Flexion YZ: " << flexions_components.at(2) << endl << endl;
  }

  in << "-----------------------------------------------------" << endl;
}*/





int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv);
    QApplication a(argc, argv);
    MainWindow app(argc, argv);
    app.show();
    return a.exec();
}
