#ifndef MISSION_BUILDER_H
#define MISSION_BUILDER_H

#include <fstream>
#include <iostream>
#include <inspector_gcs/QGCGeo.h>
// #include <json.hpp>
#include <inspector_gcs/get_from_json.h>
#include <QLineF>
#include <QPolygonF>
#include <QGeoCoordinate> 
#include <nav_msgs/Path.h>
#include <geographic_msgs/GeoPath.h>


class MissionBuilder {
    public:
        void _buildTransects(QGeoCoordinate BaseCoordinate, QList<QGeoCoordinate> _PolygonCoordinates, double _gridAngle, double _gridSpacing, QList<QLineF>& resultLines, QList<QList<QGeoCoordinate>>& resultTransects);
        void _buildMission(int nDrones, QList<QLineF>& resultLines, QList<QList<QPointF>>& droneWayPoints);
        void _heightDistribution(int nDrones, std::vector<int>& h_d, int h_mission, int h_minima, int d_seg);
        std::vector<nav_msgs::Path> _createMissionPaths(QList<QList<QPointF>> droneWayPoints, std::vector<int> h_posic, int h_barrido);
        std::vector<geographic_msgs::GeoPath> _createMissionPathsGeo(QList<QList<QGeoCoordinate>> droneWayPoints, std::vector<int> h_posic, int h_barrido);
        void _changeMissionAltitude(std::vector<geographic_msgs::GeoPath>& vectorPaths, std::vector<double> h_barrido);


        enum EntryLocation {
            EntryLocationFirst,
            EntryLocationTopLeft = EntryLocationFirst,
            EntryLocationTopRight,
            EntryLocationBottomLeft,
            EntryLocationBottomRight,
            EntryLocationLast = EntryLocationBottomRight
        };  

    private:
        double _wpsDistance (QPointF point1, QPointF point2);
        QPointF _intermediatePoint (QPointF point1, QPointF point2, double distance);
        QPointF _rotatePoint(const QPointF& point, const QPointF& origin, double angle);
        void _intersectLinesWithPolygon(const QList<QLineF>& lineList, const QPolygonF& polygon, QList<QLineF>& resultLines);
        void _adjustLineDirection(const QList<QLineF>& lineList, QList<QLineF>& resultLines);
        // void _optimizeTransectsForShortestDistance(const QGeoCoordinate& distanceCoord, QList<QList<QGeoCoordinate>>& transects);
        void _reverseTransectOrder(QList<QList<QGeoCoordinate>>& transects);
        void _reverseInternalTransectPoints(QList<QList<QGeoCoordinate>>& transects);
        void _adjustTransectsToEntryPointLocation(QList<QList<QGeoCoordinate>>& transects);
        void _findEntryPoint(QGeoCoordinate BaseCoordinate, QList<QGeoCoordinate> _PolygonCoordinates);
        void _reverseLineOrder(QList<QLineF>& lines);
        void _reverseInternalLinePoints(QList<QLineF>& lines);
        void _adjustLinesToEntryPointLocation(QList<QLineF>& lines);
        double _clampGridAngle90(double gridAngle);

        int _entryPoint;
};

#endif  // MISSION_BUILDER_H