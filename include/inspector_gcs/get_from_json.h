#ifndef GET_FROM_JSON_H
#define GET_FROM_JSON_H

#include <json.hpp>
#include <fstream>
#include <iostream>

#include <QObject>
#include <QList>
#include <QLineF>
#include <QGeoCoordinate> 
#include <QString>


using json = nlohmann::json;



class GetFromJson {
    public:
        // bool GetCoordinates(std::list<QGeoCoordinate>& _PolygonCoordinates);
        bool GetCoordinates(QList<QGeoCoordinate>& _PolygonCoordinates, double& _h_c, QString _qfilelocation);
        // double GetFlightAltitude(QString _qfilelocation);
        // bool GetFlightCharacteristics();

};


#endif  // GET_FROM_JSON_H