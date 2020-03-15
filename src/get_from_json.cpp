#include <inspector_gcs/get_from_json.h>

GetFromJson::GetFromJson(std::string& _filelocation) {
  filelocation = _filelocation;
  mission_js = json::array({});

  std::ifstream file(filelocation.c_str());
  file >> mission_js;
};

bool GetFromJson::LoadFile() {
  std::ifstream file(filelocation.c_str());
  file >> mission_js;
}

bool GetFromJson::GetCoordinates(QList<QGeoCoordinate>& _PolygonCoordinates) {

  auto p = mission_js["perimeter"];
  // std::cout << p[1]["latitude"] << std::endl;
  // std::cout << mission_js["perimeter"][1]["latitude"] << std::endl;
  std::cout << "perimeter points: " << mission_js["perimeter"].size() << std::endl;

  // Coordinates from file.json to _PolygonCoordinates
  for(int i=0; i<mission_js["perimeter"].size(); i++)
  {
    std::cout << mission_js["perimeter"][i]["latitude"] << mission_js["perimeter"][i]["longitude"] << std::endl;  
    _PolygonCoordinates.push_back(QGeoCoordinate{mission_js["perimeter"][i]["latitude"],mission_js["perimeter"][i]["longitude"]});
   };
  
  return true;
};

double GetFromJson::GetFlightAltitude() {
  return mission_js["flight characteristics"]["flight altitude"];
};

bool GetFromJson::DifferentFlightAltitudes() {
  try {
    if (mission_js["flight characteristics"].contains("diferent flight altitudes")) {
      std::cout << "mission file contains 'diferent flight altitudes'" << std::endl;
      if (mission_js["flight characteristics"]["diferent flight altitudes"] == "True") {return true;}
      else {return false;}
    }
    else {
      std::cout << "mission file doesn't contain 'diferent flight altitudes'" << std::endl;
      return false;
    }
  }
  catch (const std::exception e) {
    std::cout << "Exception: " << e.what() << std::endl;
    return false;
  }
};

std::vector<double> GetFromJson::GetDifferentFlightAltitudes() {
  return mission_js["flight characteristics"]["list of flight altitudes"];
};


double GetFromJson::GetFlightDirection() {
  return mission_js["flight characteristics"]["flight direction"];
};

double GetFromJson::GetLongitudinalOverlapMin() {
  return mission_js["flight characteristics"]["longitudinal overlap (minimum)"];
};
double GetFromJson::GetLongitudinalOverlapIdeal() {
  return mission_js["flight characteristics"]["longitudinal overlap (ideal)"];
};

double GetFromJson::GetTransverseOverlapMin() {
  return mission_js["flight characteristics"]["transverse  overlap (minimum)"];
};
double GetFromJson::GetTransverseOverlapIdeal() {
  return mission_js["flight characteristics"]["transverse  overlap (ideal)"];
};