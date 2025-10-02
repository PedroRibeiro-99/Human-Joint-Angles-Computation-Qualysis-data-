#ifndef QUALYSISMOVEDATA_H
#define QUALYSISMOVEDATA_H

#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>
#include <string>
#include <vector>

using namespace std;

typedef struct{
  float x;
  float y;
  float z;
}Position;

typedef struct{
  string name;
  Position coordinates;
}Marker;


class QualysisMoveData{
private:
  int n_frames;
  int n_markers;
  QStringList markersNames;
  vector<vector<Marker>> markersData;
public:
  QualysisMoveData();
  void resetData();
  int get_n_frames();
  int get_n_markers();
  int load_data_file(QString fileName);
  void get_frame_data(vector<Marker> &markers_frame_date, int frame);
  void getMarkerData(Marker &marker, int markerID, int frame);
  int getMarkerID(string markerName);
  vector<int> getMarkersIDs(vector<string> markersNames);
};

#endif // QUALYSISMOVEDATA_H
