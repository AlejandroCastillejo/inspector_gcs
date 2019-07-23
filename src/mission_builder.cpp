#include <ros/ros.h>

#include <inspector_gcs/mission_builder.h>
#include <QtDebug>

void MissionBuilder::_buildTransects(QGeoCoordinate BaseCoordinate, QList<QGeoCoordinate> _PolygonCoordinates, double _gridAngle, double _gridSpacing, QList<QLineF>& _resultLines, QList<QList<QGeoCoordinate>>& resultTransects)
{
    // double _gridSpacing = 15;

    QList<QPointF> polygonPoints;
    // QGeoCoordinate tangentOrigin = _PolygonCoordinates.pathModel().value<QGCCoordinate*>(0)->coordinate();
    QGeoCoordinate tangentOrigin = _PolygonCoordinates[0];
    // qCDebug(SurveyLog) << "_rebuildTransectsPhase1 Convert polygon to NED - _PolygonCoordinates.count():tangentOrigin" << _PolygonCoordinates.count() << tangentOrigin;
    for (int i=0; i<_PolygonCoordinates.count(); i++) {
        double y, x, down;
        // QGeoCoordinate vertex = _PolygonCoordinates.pathModel().value<QGCCoordinate*>(i)->coordinate();
        QGeoCoordinate vertex = _PolygonCoordinates[i];
        if (i == 0) {
            // This avoids a nan calculation that comes out of convertGeoToNed
            x = y = 0;
        } else {
            // convertGeoToNed(vertex, tangentOrigin, &y, &x, &down);
            convertGeoToNed(vertex, tangentOrigin, &x, &y, &down);
        }
        polygonPoints += QPointF(x, y);
        // qCDebug(SurveyLog) << "_rebuildTransectsPhase1 vertex:x:y" << vertex << polygonPoints.last().x() << polygonPoints.last().y();
    }

    QList<QPointF>::iterator iter;
    std::cout << "\n NED Coordinates \n" << std::endl;
    for( iter = polygonPoints.begin(); iter != polygonPoints.end(); iter++)
    {
       std::cout << iter->x() << "    " <<iter->y() << std::endl;
    };

    // Generate transects

    // nlohmann::json js = nlohmann::json::array({});
    // std::ifstream file("mission_parameters.json");
    // file >> js;
    // std::cout << "\ngridAngle: " << js["gridAngle"] << std::endl;
    // std::cout << "gridSpacing: " << js["gridSpacing"] << "\n" << std::endl;

    // double _gridAngle = js["gridAngle"];

    // double _gridAngle = _get_from_json.GetFlightDirection() - 90;
    std::cout << "\ngridAngle: " << _gridAngle << std::endl;
    
    _gridAngle = _clampGridAngle90(_gridAngle);
    
    QPolygonF polygon;
    for (int i=0; i<polygonPoints.count(); i++) {
        // qDebug() << "Vertex" << polygonPoints[i];
        // qCDebug(SurveyLog) << "Vertex" << polygonPoints[i];
        polygon << polygonPoints[i];
    }
    polygon << polygonPoints[0];

    qDebug() << "polygon" << polygon << endl;

    QRectF boundingRect = polygon.boundingRect();
    qDebug() << "boundingRect" << boundingRect;

    QPointF boundingCenter = boundingRect.center();
    qDebug() << "boundingCenter" << boundingCenter;

    // qDebug() << "Bounding rect" << boundingRect.topLeft().x() << boundingRect.topLeft().y() << boundingRect.bottomRight().x() << boundingRect.bottomRight().y();
    
    // Create set of rotated parallel lines within the expanded bounding rect. Make the lines larger than the
    // bounding box to guarantee intersection.

    QList<QLineF> lineList;

    // Transects are generated to be as long as the largest width/height of the bounding rect plus some fudge factor.
    // This way they will always be guaranteed to intersect with a polygon edge no matter what angle they are rotated to.
    // They are initially generated with the transects flowing from west to east and then points within the transect north to south.
    double maxWidth = qMax(boundingRect.width(), boundingRect.height()) + 2000.0;
    double halfWidth = maxWidth / 2.0;
    double transectX = boundingCenter.x() - halfWidth;
    double transectXMax = transectX + maxWidth;
    // qDebug() << boundingRect.width() << boundingRect.height() << maxWidth << transectX << transectXMax;

    while (transectX < transectXMax) {
        double transectYTop = boundingCenter.y() - halfWidth;
        double transectYBottom = boundingCenter.y() + halfWidth;

        lineList += QLineF(_rotatePoint(QPointF(transectX, transectYTop), boundingCenter, _gridAngle), _rotatePoint(QPointF(transectX, transectYBottom), boundingCenter, _gridAngle));
        transectX += _gridSpacing;
    }
    // qDebug() << lineList;

    // Now intersect the lines with the polygon
    QList<QLineF> intersectLines;

    _intersectLinesWithPolygon(lineList, polygon, intersectLines);

    
    QList<QLineF> resultLines;
    _adjustLineDirection(intersectLines, resultLines);

    // qDebug() << endl << "intersectLines" << endl << resultLines << endl;
    // qDebug() << endl << "resultLines" << endl << resultLines << endl;


    // Adjust to lawnmower pattern
    bool reverseVertices = false;
    for (int i=0; i<resultLines.count(); i++) {
        // We must reverse the vertices for every other transect in order to make a lawnmower pattern
        QLineF resultLineVertices = resultLines[i];
        if (reverseVertices) {
            reverseVertices = false;
            QLineF reversedVertices = QLineF(resultLines[i].p2(),resultLines[i].p1());
            resultLineVertices = reversedVertices;
        } else {
            reverseVertices = true;
        }
        resultLines[i] = resultLineVertices;
    }
    _resultLines = resultLines;
    // resultTransects = transects;

    _findEntryPoint(BaseCoordinate, _PolygonCoordinates);
    _adjustLinesToEntryPointLocation(_resultLines);


    // qDebug() << endl << "resultLines ajusted" << endl << resultLines << endl;
    

    // // Convert from NED to Geo
    // QList<QList<QGeoCoordinate>> transects;
    // foreach (const QLineF& line, resultLines) {
    //     QGeoCoordinate          coord;
    //     QList<QGeoCoordinate>   transect;

    //     convertNedToGeo(line.p1().y(), line.p1().x(), 0, tangentOrigin, &coord);
    //     transect.append(coord);
    //     convertNedToGeo(line.p2().y(), line.p2().x(), 0, tangentOrigin, &coord);
    //     transect.append(coord);

    //     transects.append(transect);
    // }
    // qDebug() << endl << "transects" << endl << transects << endl;


    // if (refly) {
    //     _optimizeTransectsForShortestDistance(_transects.last().last().coord, transects);
    // }

    // if (_flyAlternateTransectsFact.rawValue().toBool()) {
    //     QList<QList<QGeoCoordinate>> alternatingTransects;
    //     for (int i=0; i<transects.count(); i++) {
    //         if (!(i & 1)) {
    //             alternatingTransects.append(transects[i]);
    //         }
    //     }
    //     for (int i=transects.count()-1; i>0; i--) {
    //         if (i & 1) {
    //             alternatingTransects.append(transects[i]);
    //         }
    //     }
    //     transects = alternatingTransects;
    // }


    // qDebug() << endl << "transectVertices" << endl << transects << endl;
    // for (int k=0; k<transects.count()-1; k++) {
    //     qDebug() << endl << transects[k] << endl;
    // }


    // // Convert to CoordInfo transects and append to _transects
    // foreach (const QList<QGeoCoordinate>& transect, transects) {
    //     QGeoCoordinate                                  coord;
    //     // QList<TransectStyleComplexItem::CoordInfo_t>    coordInfoTransect;
    //     // TransectStyleComplexItem::CoordInfo_t           coordInfo;
    //     QList<CoordInfo_t>    coordInfoTransect;
    //     CoordInfo_t           coordInfo;


    //     coordInfo = { transect[0], CoordTypeSurveyEdge };
    //     coordInfoTransect.append(coordInfo);
    //     coordInfo = { transect[1], CoordTypeSurveyEdge };
    //     coordInfoTransect.append(coordInfo);

        
    //     // Extend the transect ends for turnaround
    //     // if (_hasTurnaround()) {
    //     if (1) {
    //         QGeoCoordinate turnaroundCoord;
    //         // double turnAroundDistance = _turnAroundDistanceFact.rawValue().toDouble();
    //         double turnAroundDistance = 5;

    //         double azimuth = transect[0].azimuthTo(transect[1]);
    //         turnaroundCoord = transect[0].atDistanceAndAzimuth(-turnAroundDistance, azimuth);
    //         turnaroundCoord.setAltitude(qQNaN());
    //         // TransectStyleComplexItem::CoordInfo_t coordInfo = { turnaroundCoord, CoordTypeTurnaround };
    //         CoordInfo_t coordInfo = { turnaroundCoord, CoordTypeTurnaround };
    //         coordInfoTransect.prepend(coordInfo);
    //         wp = turnaroundCoord;
    //         WayPoints.prepend(wp);
        

    //         azimuth = transect.last().azimuthTo(transect[transect.count() - 2]);
    //         turnaroundCoord = transect.last().atDistanceAndAzimuth(-turnAroundDistance, azimuth);
    //         turnaroundCoord.setAltitude(qQNaN());
    //         coordInfo = { turnaroundCoord, CoordTypeTurnaround };
    //         coordInfoTransect.append(coordInfo);
    //         wp = turnaroundCoord;
    //         WayPoints.append(wp);
    //     }

    //     _transects.append(coordInfoTransect);
    // }
    //     qDebug() << fixed << qSetRealNumberPrecision(10);
    //     qDebug() << "waipoints" << endl << WayPoints << endl;

}

// Calculate the total survey length and distribute lines between diferent drones
void MissionBuilder::_buildMission(int nDrones, QList<QLineF>& resultLines, QList<QList<QPointF>>& droneWayPoints) {

    QList<QPointF> WayPoints;
    for(int i=0; i<resultLines.count(); i++) {
        WayPoints.append(resultLines[i].p1());
        WayPoints.append(resultLines[i].p2());
    }
    // qDebug() << "resultLines" << endl << resultLines << endl;
    // // qDebug() << "waypoints" << endl << WayPoints <<endl;

    double totalLength = 0;
    double f = 0; 
        for (int i=0; i<WayPoints.count()-1; i++) {
            totalLength += _wpsDistance(WayPoints[i], WayPoints[i+1]);
            // qDebug()<< i;
            // qDebug()<< totalLength;
        }

    qDebug() << endl << "totalLenght:  " << totalLength << endl;

    double eachLength = totalLength/nDrones;
    qDebug() << "each drone Lenght:  " << eachLength << endl;


    // QList<QList<QPointF>> droneWayPoints;
    QList<QPointF> idroneWayPoints;
    QPointF interWp;
    int lastwp=0;
    for (int i=0; i<nDrones; i++) {
        // qDebug()<< "drone" << i;
        int j=lastwp;
        double distanceLeft;
        double f=0;
        if(i != 0) {
            idroneWayPoints.append(interWp);
            f+=_wpsDistance(interWp, WayPoints[j]);
            // qDebug()<< "inter wp" << j;
            // qDebug()<< "length" << f;
        }
        while(f<=eachLength && j<WayPoints.count()-1) {
            f+=_wpsDistance(WayPoints[j], WayPoints[j+1]);
            // qDebug()<< "wp" << j;
            // qDebug()<< "length" << _wpsDistance(WayPoints[j], WayPoints[j+1]);
            // qDebug()<< "total length" << f;
            idroneWayPoints.append(WayPoints[j]);
            j++;
        }
        if(j<=WayPoints.count()-1) {
            distanceLeft = eachLength - f + _wpsDistance(WayPoints[j-1], WayPoints[j]);
            // qDebug() << j;
            // qDebug() << "distanceLeft" << distanceLeft;
            interWp = _intermediatePoint(WayPoints[j-1], WayPoints[j], distanceLeft);
            // qDebug() << "intermediatePoint" << interWp;
        }
        idroneWayPoints.append(interWp);
        droneWayPoints.append(idroneWayPoints);

    //Test
        // double e=0;
        // for(int i=0; i<idroneWayPoints.count()-1; i++){
        //     e += _wpsDistance(idroneWayPoints[i], idroneWayPoints[i+1]);
        // }
        //     qDebug() << "suma waypoints length" << e;

        idroneWayPoints.clear();
        lastwp=j;

        // qDebug() << droneWayPoints[i];
        // qDebug()<< endl;
    }

}

void MissionBuilder::_heightDistribution(int nDrones, std::vector<int>& h_d, int h_mission, int h_min_seg, int d_seg) {
    h_d.clear();
    int h_d_min = h_mission;
    int h_d_max = h_mission;
    for (int i=0; i<nDrones; i++) {
        h_d_min -= d_seg;
        h_d_max += d_seg;
        if(h_d_min > h_min_seg) {
            //  h_d[i]= h_d_min;
             h_d.push_back(h_d_min);
             i++;
        }
        // h_d[i] = h_d_max;
        h_d.push_back(h_d_max);
        
    }
    qDebug() << endl << "positioning heights";
    for (int i=0; i<nDrones; i++) {
        qDebug() << h_d[i];
    }
}


// void MissionBuilder::_createMissionPaths(QList<QList<QPointF>> droneWayPoints, std::vector<int> h_posic, int h_barrido, std::vector<nav_msgs::Path>& missionPath){
std::vector<nav_msgs::Path> MissionBuilder::_createMissionPaths(QList<QList<QPointF>> droneWayPoints, std::vector<int> h_posic, int h_barrido)
{
    std::vector<nav_msgs::Path> missionPaths;
    for (QList<QPointF> iDroneWayPoints : droneWayPoints) 
    {
        nav_msgs::Path iPath;
        iPath.header.frame_id = "map";
        for (QPointF Waypoint : iDroneWayPoints) {
            geometry_msgs::PoseStamped wp;
            wp.header.frame_id = "map";
            wp.header.stamp = ros::Time::now();
            wp.pose.position.x = Waypoint.x();
            wp.pose.position.y = Waypoint.y();
            wp.pose.position.z = h_barrido;

            iPath.poses.push_back(wp);
        // std::cout << wp << std::endl;
        }
        missionPaths.push_back(iPath);
    }

    return missionPaths;
}

std::vector<geographic_msgs::GeoPath> MissionBuilder::_createMissionPathsGeo(QList<QList<QGeoCoordinate>> droneWayPoints, std::vector<int> h_posic, int h_barrido)
{
    ROS_INFO("creating paths geo");
    std::vector<geographic_msgs::GeoPath> missionPaths;
    for (QList<QGeoCoordinate> iDroneWayPoints : droneWayPoints) 
    {
        geographic_msgs::GeoPath iPath;
        for (QGeoCoordinate Waypoint : iDroneWayPoints) {
            // qDebug()<< Waypoint << endl;
            geographic_msgs::GeoPoseStamped wp;
            wp.pose.position.latitude = Waypoint.latitude();
            wp.pose.position.longitude = Waypoint.longitude();
            wp.pose.position.altitude = h_barrido;

            iPath.poses.push_back(wp);
            // std::cout << wp << std::endl;
 
        }
        missionPaths.push_back(iPath);
    }

    return missionPaths;
}


double MissionBuilder::_wpsDistance (QPointF point1, QPointF point2) {
    double dx = point2.x() - point1.x();
    double dy = point2.y() - point1.y();
    return sqrt(dx*dx+dy*dy);
}

QPointF MissionBuilder::_intermediatePoint (QPointF point1, QPointF point2, double distance) {
    double dx = point2.x() - point1.x();
    double dy = point2.y() - point1.y();
    double x = point1.x() + dx*distance/_wpsDistance(point1, point2);
    double y = point1.y() + dy*distance/_wpsDistance(point1, point2);
    return QPointF(x,y);
}


double MissionBuilder::_clampGridAngle90(double _gridAngle)
{
    // Clamp grid angle to -90<->90. This prevents transects from being rotated to a reversed order.
    if (_gridAngle > 90.0) {
        _gridAngle -= 180.0;
    } else if (_gridAngle < -90.0) {
        _gridAngle += 180;
    }
    return _gridAngle;
}

QPointF MissionBuilder::_rotatePoint(const QPointF& point, const QPointF& origin, double angle)
{
    QPointF rotated;
    double radians = (M_PI / 180.0) * angle;

    rotated.setX(((point.x() - origin.x()) * cos(radians)) - ((point.y() - origin.y()) * sin(radians)) + origin.x());
    rotated.setY(((point.x() - origin.x()) * sin(radians)) + ((point.y() - origin.y()) * cos(radians)) + origin.y());

    return rotated;
}


void MissionBuilder::_intersectLinesWithPolygon(const QList<QLineF>& lineList, const QPolygonF& polygon, QList<QLineF>& resultLines)
// void MissionsBuilder::_intersectLinesWithPolygon(const QList<QLineF>& lineList, const QPolygonF& polygon, QList<QLineF>& resultLines)
{
    for (int i=0; i<lineList.count(); i++) {
        const QLineF& line = lineList[i];
        QList<QPointF> intersections;

        // Intersect the line with all the polygon edges
        for (int j=0; j<polygon.count()-1; j++) {
            QPointF intersectPoint;
            QLineF polygonLine = QLineF(polygon[j], polygon[j+1]);
            if (line.intersect(polygonLine, &intersectPoint) == QLineF::BoundedIntersection) {
                if (!intersections.contains(intersectPoint)) {
                    intersections.append(intersectPoint);
                }
            }
        }

        // We now have one or more intersection points all along the same line. Find the two
        // which are furthest away from each other to form the transect.
        if (intersections.count() > 1) {
            QPointF firstPoint;
            QPointF secondPoint;
            double currentMaxDistance = 0;

            for (int i=0; i<intersections.count(); i++) {
                for (int j=0; j<intersections.count(); j++) {
                    QLineF lineTest(intersections[i], intersections[j]);
                    \
                    double newMaxDistance = lineTest.length();
                    if (newMaxDistance > currentMaxDistance) {
                        firstPoint = intersections[i];
                        secondPoint = intersections[j];
                        currentMaxDistance = newMaxDistance;
                    }
                }
            }

            resultLines += QLineF(firstPoint, secondPoint);
        }
    }
}

/// Adjust the line segments such that they are all going the same direction with respect to going from P1->P2
void MissionBuilder::_adjustLineDirection(const QList<QLineF>& lineList, QList<QLineF>& resultLines)
// void MissionsBuilder::_adjustLineDirection(const QList<QLineF>& lineList, QList<QLineF>& resultLines)
{
    qreal firstAngle = 0;
    for (int i=0; i<lineList.count(); i++) {
        const QLineF& line = lineList[i];
        QLineF adjustedLine;

        if (i == 0) {
            firstAngle = line.angle();
        }

        if (qAbs(line.angle() - firstAngle) > 1.0) {
            adjustedLine.setP1(line.p2());
            adjustedLine.setP2(line.p1());
        } else {
            adjustedLine = line;
        }

        resultLines += adjustedLine;
    }
}

void MissionBuilder::_adjustLinesToEntryPointLocation(QList<QLineF>& lines)
{
    if (lines.count() == 0) {
        return;
    }

    bool reversePoints = false;
    bool reverseTransects = false;

    // if (_entryPoint == EntryLocationBottomLeft || _entryPoint == EntryLocationBottomRight) {
    //     reversePoints = true;
    // }
    // if (_entryPoint == EntryLocationTopRight || _entryPoint == EntryLocationBottomRight) {
    //     reverseTransects = true;
    // }
    if (_entryPoint == EntryLocationTopLeft || _entryPoint == EntryLocationBottomLeft) {
        reversePoints = true;
    }
    if (_entryPoint == EntryLocationTopRight || _entryPoint == EntryLocationTopLeft) {
        reverseTransects = true;
    }
    
    if (reversePoints) {
        // qCDebug(SurveyComplexItemLog) << "_adjustTransectsToEntryPointLocation Reverse Points";
        _reverseInternalLinePoints(lines);
    }
    if (reverseTransects) {
        // qCDebug(SurveyComplexItemLog) << "_adjustTransectsToEntryPointLocation Reverse Transects";
        _reverseLineOrder(lines);
    }
    // qCDebug(SurveyComplexItemLog) << "_adjustTransectsToEntryPointLocation Modified entry point:entryLocation" << transects.first().first() << _entryPoint;
}


/// Reverse the order of the transects. First transect becomes last and so forth.
void MissionBuilder::_reverseLineOrder(QList<QLineF>& lines)
{
    qDebug() << "reverseLineOrder" << endl;

    QList<QLineF> rgReversedLines;
    for (int i=lines.count() - 1; i>=0; i--) {
        rgReversedLines.append(lines[i]);
    }
    qDebug() << "lines:" << lines << endl;
    qDebug() << "ReversedLines:" << rgReversedLines << endl;

    lines = rgReversedLines;
}

/// Reverse the order of all points withing each transect, First point becomes last and so forth.
void MissionBuilder::_reverseInternalLinePoints(QList<QLineF>& lines)
{
    qDebug() << "reverseInternalLinePoints" << endl;

    QList<QLineF> reversedLines;

    for (int i=0; i<lines.count(); i++) {
        reversedLines.append( QLineF(lines[i].p2(), lines[i].p1()) );
    }

    qDebug() << "lines:" << lines << endl;
    qDebug() << "reversedLines:" << reversedLines << endl;

    lines = reversedLines;
}


void MissionBuilder::_findEntryPoint(QGeoCoordinate BaseCoordinate, QList<QGeoCoordinate> _PolygonCoordinates) 
{
    QGeoCoordinate polygonCenter;
    double sumLatitude = 0;
    double sumLongitude = 0;
    for (int i=0; i<_PolygonCoordinates.count(); i++) {
        sumLatitude += _PolygonCoordinates[i].latitude();
        sumLongitude += _PolygonCoordinates[i].longitude();
    }
    polygonCenter.setLatitude(sumLatitude / _PolygonCoordinates.count());
    polygonCenter.setLongitude(sumLongitude / _PolygonCoordinates.count());

    qDebug() << "polygonCenter: " << polygonCenter << endl;
    qDebug() << "baseCoordinate: " << BaseCoordinate << endl;

    if (polygonCenter.latitude() < BaseCoordinate.latitude()) {
        if (polygonCenter.longitude() < BaseCoordinate.longitude()) {
            _entryPoint = EntryLocationTopRight;
            // _entryPoint = EntryLocationBottomRight;
            std::cout << "entryPoint: EntryLocationTopRight" << std::endl;
        } else {
            _entryPoint = EntryLocationTopLeft; 
            // _entryPoint = EntryLocationTopRight;
            std::cout << "entryPoint: EntryLocationTopLeft" << std::endl;
        }
    } else {
        if (polygonCenter.longitude() < BaseCoordinate.longitude()) {
            _entryPoint = EntryLocationBottomRight;
            // _entryPoint = EntryLocationBottomLeft;            
            std::cout << "entryPoint: EntryLocationBottomRight" << std::endl;
        } else {
            _entryPoint = EntryLocationBottomLeft;  
            // _entryPoint = EntryLocationTopLeft;            
            std::cout << "entryPoint: EntryLocationBottomLeft" << std::endl;
        }
    }
    // qDebug() << "entryPoint" << _entryPoint << endl;
    std::cout << "entryPoint: " << _entryPoint << std::endl;
    
}


// /// Reverse the order of the transects. First transect becomes last and so forth.
// void MissionBuilder::_reverseTransectOrder(QList<QList<QGeoCoordinate>>& transects)
// {
//     QList<QList<QGeoCoordinate>> rgReversedTransects;
//     for (int i=transects.count() - 1; i>=0; i--) {
//         rgReversedTransects.append(transects[i]);
//     }
//     transects = rgReversedTransects;
// }

// /// Reverse the order of all points withing each transect, First point becomes last and so forth.
// void MissionBuilder::_reverseInternalTransectPoints(QList<QList<QGeoCoordinate>>& transects)
// {
//     for (int i=0; i<transects.count(); i++) {
//         QList<QGeoCoordinate> rgReversedCoords;
//         QList<QGeoCoordinate>& rgOriginalCoords = transects[i];
//         for (int j=rgOriginalCoords.count()-1; j>=0; j--) {
//             rgReversedCoords.append(rgOriginalCoords[j]);
//         }
//         transects[i] = rgReversedCoords;
//     }
// }


