#ifndef GCS_SERVICES_H
#define GCS_SERVICES_H

#include <thread>
#include <QtDebug>

#include <ros/ros.h>
#include <ros/package.h>
#include <inspector_gcs/get_from_json.h>
#include <inspector_gcs/mission_builder.h>
#include <inspector_gcs/camera_calc.h>
#include <inspector_gcs/gcsCreateMission.h>
#include <inspector_gcs/gcsSendMission.h>
#include <inspector_gcs/MissionService.h>
#include <inspector_gcs/uavLink.h>
// #include <inspector_gcs/UAV_Link.h>

// #include <parametros.h>

#include <sensor_msgs/NavSatFix.h>

#include <visualization_msgs/Marker.h>


// std::string package_path = ros::package::getPath("inspector_gcs");
// std::string mission_file_location = package_path + "/json_files/mission_file.json";

// std::string rgb_camera_name = "Sony a6000";
// std::string thermal_camera_name = "WIRIS 2nd gen";

class GcsServices {

    public:
        GcsServices(ros::NodeHandle &_nh, std::vector<std::string> &_uav_list, QList<QList<QPointF>> &_droneWayPointsNED);


    private:
        int n; // number of uavs
        std::vector<int> h_d; // positioning heigth vector

        // ToDo: get following variables by rosparameters or text-file
        int h_min = 10; //minumun flight altitude
        int d = 6; //minimun distance
        //
        // std::string package_path = ros::package::getPath("inspector_gcs");
        // std::string mission_file_location = package_path + "/json_files/mission_file.json";
        
        std::string rgb_camera_name = "Sony a6000";
        std::string thermal_camera_name = "WIRIS 2nd gen";
        //

        ros::NodeHandle *p_nh;
        
        GetFromJson get_from_json;
        MissionBuilder mission_builder;

         // CameraCalc Objects
        CameraCalc rgb_camera;
        CameraCalc thermal_camera;

        // ROS Services
        ros::ServiceServer uav_link_srv, create_mission_srv, send_mission_srv;

        bool uav_link_server_cb(inspector_gcs::uavLink::Request &req, inspector_gcs::uavLink::Response &res);
        bool create_mission_cb(inspector_gcs::gcsCreateMission::Request &req, inspector_gcs::gcsCreateMission::Response &res);
        bool send_mission_cb(inspector_gcs::gcsSendMission::Request &req, inspector_gcs::gcsSendMission::Response &res);
        //
        // void waypoints_publisher(QList<QList<QPointF>> _droneWayPointsNED);
        std::thread vis_thread;
        // void visualization_thread(QList<QList<QPointF>> _droneWayPointsNED, std::vector<int> _h_d, int _h_c);
        void visualization_thread(QList<QList<QPointF>> _droneWayPointsNED, QList<QList<QGeoCoordinate>> _droneWayPointsGeo, std::vector<int> _h_d, int _h_c);

        // ROS Subscribers
        ros::Subscriber uav1_coordinate_sub;
        // void uav_pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        sensor_msgs::NavSatFix uav1_coordinate;
        //

        std::vector<std::string> uav_list;
        std::vector<std::string>* p_uav_list;  // Pointer to uav_list

        QGeoCoordinate BaseCoordinate;
        QList<QGeoCoordinate> PolygonCoordinates;
        double gridAngle;
        double gridSpacing;
        double h_c; //flight altitude

        // QList<QGeoCoordinate> WaiPoints;
        QList<QLineF> resultLinesNED;
        QList<QList<QGeoCoordinate>> resultTransectsGeo;
        QList<QList<QPointF>> droneWayPointsNED;
        QList<QList<QGeoCoordinate>> droneWayPointsGeo;

        std::vector<nav_msgs::Path> missionPaths;
        std::vector<geographic_msgs::GeoPath> missionPathsGeo;

        bool mission_created = false;
};

#endif // GCS_SERVICES_H