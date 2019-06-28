#ifndef GET_FROM_JSON_H
#define GET_FROM_JSON_H

#include <json.hpp> // single-include file for json parsing
#include <fstream>
#include <iostream>

#include <QObject>
#include <QList>
#include <QLineF>
#include <QGeoCoordinate> 
// #include <QString>

// std::cout << "test" << std::endl;

using json = nlohmann::json;


class GetFromJson {
    public:
        GetFromJson(std::string& _filelocation);
        // bool GetCoordinates(std::list<QGeoCoordinate>& _PolygonCoordinates);
        // GetPlantName();
        bool GetCoordinates(QList<QGeoCoordinate>& _PolygonCoordinates);
        double GetFlightAltitude();
        // bool GetFlightCharacteristics();
        double GetFlightDirection();
        double GetLongitudinalOverlapMin();
        double GetLongitudinalOverlapIdeal();
        double GetTransverseOverlapMin();
        double GetTransverseOverlapIdeal();

    private:
        json mission_js;
};


#endif  // GET_FROM_JSON_H