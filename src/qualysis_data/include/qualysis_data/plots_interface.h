#ifndef PLOTS_INTERFACE_H
#define PLOTS_INTERFACE_H

#include <QDialog>
#include <QFileDialog>
#include "joint.h"
#include "qcustomplot.h"

namespace Ui {
class PlotsInterface;
}

class PlotsInterface : public QDialog
{
  Q_OBJECT

public:
  explicit PlotsInterface(QWidget *parent = nullptr);
  ~PlotsInterface();
  void reset_plots();
  void update_plots(int frame, vector<Joint*> joints);
  void draw_plots();

private Q_SLOTS:
  void resetZoom();

private:
  Ui::PlotsInterface *ui;
  vector<QCustomPlot*> plots;
  vector<string> plots_names;
  //Joint jR_arm, jR_forearm, jR_wrist;
};

#endif // PLOTS_INTERFACE_H
