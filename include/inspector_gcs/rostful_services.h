#ifndef ROSTFUL_SERVICES
#define ROSTFUL_SERVICES

#include <ros/ros.h>

#include <inspector_gcs/UavListService.h>
#include <inspector_gcs/StartMission.h>


class RostfulServices {

    public:
        RostfulServices(ros::NodeHandle &_nh, std::vector<std::string> &_uav_list);
        // ~RostfulServices();
        
        
        // Ros Services
        ros::ServiceServer uav_list_server;
        ros::ServiceServer start_mission_server;




        bool uav_list_server_cb(inspector_gcs::UavListService::Request &req, inspector_gcs::UavListService::Response &res);
        bool start_mission_server_cb(inspector_gcs::StartMission::Request &req, inspector_gcs::StartMission::Response &res);
    private:

        // std::vector* myvector_ptr;
        std::vector<std::string>* _uav_list_ptr;
        // std::vector<std::string> uav_list;
};




#endif // ROSTFUL_SERVICES