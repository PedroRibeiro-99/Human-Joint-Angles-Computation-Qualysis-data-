#include "../include/qualysis_data/qualysis_move_data.h"

QualysisMoveData::QualysisMoveData()
{

}

void QualysisMoveData::resetData(){
  this->numberOfFrames = 0;
  this->numberOfMarkers = 0;
  this->markersNames.clear();
  this->markersData.clear();
}

int QualysisMoveData::getFrequency(){
  return this->frequency;
}


int QualysisMoveData::getNumberOfFrames(){
  return this->numberOfFrames;
}

int QualysisMoveData::getNumberOfMarkers(){
  return this->numberOfMarkers;
}

QStringList QualysisMoveData::getMarkersNames(){
  return this->markersNames;
}


int QualysisMoveData::loadDataFile(QString fileName){
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
    if (row == 0) this->numberOfFrames = fields[1].toInt(); //extract number of frames
    else if(row == 2)  this->numberOfMarkers = fields[1].toInt(); //extract number of markers
    else if(row == 3) this->frequency = fields[1].toInt();
    else if(row == 9) //extract markers names
      for(int column = 1; column < numberOfMarkers+1; column++)
        this->markersNames.push_back(fields[column]);
    else if(row == 11){ //finds the column that begins the positions data
      for(int column = 0 ; column < fields.size() ; column++)
        if(fields[column] == QString(this->markersNames.at(0) + " X")){
          positionsDataCollumnsOffset = column;
          break;
        }
    }
  }

  for(int frame = 0; frame < this->numberOfFrames ; frame++){
    line = in.readLine();
    fields = line.split('\t'); // tab delimiter
    vector<Marker> markers_frame_data;
    for(int column = 0; column < this->numberOfMarkers*3; column+=3){
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


void QualysisMoveData::getFrameData(vector<Marker> &markers_frame_date, int frame){

  markers_frame_date = this->markersData.at(frame);
}


void QualysisMoveData::getMarkerData(Marker &marker, int marker_id, int frame){
  marker = this->markersData.at(frame).at(marker_id);
}

int QualysisMoveData::getMarkerID(string markerName){
  for (int i = 0; i < this->numberOfMarkers; i++){
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
