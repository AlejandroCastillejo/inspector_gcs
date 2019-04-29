#include <inspector_gcs/get_from_json.h>



bool GetFromJson::GetCoordinates(QList<QGeoCoordinate>& _PolygonCoordinates, double& _h_c, QString _qfilelocation) {

  json js = json::array({});

  // std::ifstream file(_qfilelocation.toStdString());
  std::ifstream file("planta_karting.json");
  // std::ifstream file("pista_labs.json");
  // std::ifstream file("planta_utrera.json");
  // std::ifstream file("archivo2.json");
  file >> js;

  auto p = js["perimeter"];
  // std::cout << p[1]["latitude"] << std::endl;
  // std::cout << js["perimeter"][1]["latitude"] << std::endl;
  std::cout << "perimeter points: " << js["perimeter"].size() << std::endl;

  // Coordinates from file.json to _PolygonCoordinates
  for(int i=0; i<js["perimeter"].size(); i++)
  {
    std::cout << js["perimeter"][i]["latitude"] << js["perimeter"][i]["longitude"] << std::endl;
    
    _PolygonCoordinates.push_back(QGeoCoordinate{js["perimeter"][i]["latitude"],js["perimeter"][i]["longitude"]});
   };

  _h_c = js["flight characteristics"]["flight altitude"];

   return true;
};

