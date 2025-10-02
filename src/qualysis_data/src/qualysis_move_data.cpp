#include "../include/qualysis_data/qualysis_move_data.h"

QualysisMoveData::QualysisMoveData()
{

}

void QualysisMoveData::resetData(){
  this->n_frames = 0;
  this->n_markers = 0;
  this->markersNames.clear();
  this->markersData.clear();
}

int QualysisMoveData::get_n_frames(){
  return this->n_frames;
}

int QualysisMoveData::get_n_markers(){
  return this->n_markers;
}

int QualysisMoveData::load_data_file(QString fileName){
  QFile file(fileName);

  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qDebug() << "Erro ao abrir o ficheiro:" << file.errorString();
      return -1;
  }

  QTextStream in(&file);
  QString line;
  QStringList fields;

  int positionsDataCollumnsOffset = 0; // Offset indicating where the positions data columns begin
  for(int row = 0; row < 12 ; row++){
    line = in.readLine();
    fields = line.split('\t'); // tab delimiter
    if (row == 0) this->n_frames = fields[1].toInt(); //extract number of frames
    else if(row == 2)  this->n_markers = fields[1].toInt(); //extract number of markers
    else if(row == 9) //extract markers names
      for(int column = 1; column < n_markers+1; column++)
        this->markersNames.push_back(fields[column]);
    else if(row == 11){ //finds the column that begins the positions data
      for(int column = 0 ; column < fields.size() ; column++)
        if(fields[column] == QString(this->markersNames.at(0) + " X")){
          positionsDataCollumnsOffset = column;
          break;
        }
    }
  }

  for(int frame = 0; frame < this->n_frames ; frame++){
    line = in.readLine();
    fields = line.split('\t'); // tab delimiter
    vector<Marker> markers_frame_data;
    for(int column = 0; column < this->n_markers*3; column+=3){
      Marker marker;
      marker.name = markersNames[column/3].toStdString();
      marker.coordinates.x = fields[column + positionsDataCollumnsOffset].toFloat() * 0.001;
      marker.coordinates.y = fields[column + positionsDataCollumnsOffset + 1].toFloat() * 0.001;
      marker.coordinates.z = fields[column + positionsDataCollumnsOffset + 2].toFloat() * 0.001;
      markers_frame_data.push_back(marker);
    }
    this->markersData.push_back(markers_frame_data);
  }

  file.close();
  return 1;
}


void QualysisMoveData::get_frame_data(vector<Marker> &markers_frame_date, int frame){

  markers_frame_date = this->markersData.at(frame);
}


void QualysisMoveData::getMarkerData(Marker &marker, int marker_id, int frame){
  marker = this->markersData.at(frame).at(marker_id);
}

int QualysisMoveData::getMarkerID(string markerName){
  for (int i = 0; i < this->n_markers; i++){
    if(markerName == markersNames[i].toStdString())
      return i;
  }
  return -1;
}


vector<int> QualysisMoveData::getMarkersIDs(vector<string> markersNames){
  vector<int> markersIDs;

  for(string markerName : markersNames){
    int markerID = this->getMarkerID(markerName);
    markersIDs.push_back(markerID);
  }

  return markersIDs;
}
