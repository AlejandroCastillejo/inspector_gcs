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

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/NavSatFix.h>

#include <visualization_msgs/Marker.h>


class GcsServices {

    public:
        GcsServices(ros::NodeHandle &_nh, std::vector<std::string> &_uav_list);


    private:

        // %% TO DO: get following parameters by rosparameters or text-file, or include in mission_file.json
        int h_min = 10; //minumun flight altitude
        int d = 6; //minimun distance       
        std::string rgb_camera_name = "Sony a6000";
        std::string thermal_camera_name = "WIRIS 2nd gen";
        //
        ros::NodeHandle *p_nh; //pointer to nodehandler
        
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
        QGeoCoordinate BaseCoordinate;
        sensor_msgs::NavSatFix uav_coordinate;
        boost::shared_ptr<sensor_msgs::NavSatFix const> uav_coordiante_ptr; //pointer to uav coordinate message
        //
        std::thread vis_thread;
        void visualization_thread();

        int n; // number of uavs
        double h_c; //flight altitude
        std::vector<double> list_h_c; // vector with different flight altitudes
        std::vector<int> h_d; // positioning heigth vector

        std::vector<std::string> uav_list;
        std::vector<std::string>* p_uav_list;  // Pointer to uav_list

        QList<QGeoCoordinate> PolygonCoordinates;
        double gridAngle;
        double gridSpacing;

        QList<QLineF> resultLinesNED;
        QList<QList<QGeoCoordinate>> resultTransectsGeo;
        QList<QList<QPointF>> droneWayPointsNED;
        QList<QList<QGeoCoordinate>> droneWayPointsGeo;

        std::vector<nav_msgs::Path> missionPaths;
        std::vector<geographic_msgs::GeoPath> missionPathsGeo;
        std::vector<geographic_msgs::GeoPath>* p_missionPathsGeo;

        bool mission_created = false;



        // typedef uav_abstraction_layer::State ualState;

        // struct uav_state {
        //   bool connected;
        //   ros::Time last_msg;
        //   ualState ual_state;
        //   std::string adl_state;
        //   geometry_msgs::PoseStamped pose;
        //   geometry_msgs::TwistStamped velocity;
        // }uav_state_empty;

        // std::map<std::string, uav_state> map_uav_state;
        // std::map<std::string, uav_state>::iterator iter_uav;

};

#endif // GCS_SERVICES_H