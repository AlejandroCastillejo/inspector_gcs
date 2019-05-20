#ifndef ROSTFUL_SERVICES
#define ROSTFUL_SERVICES

#include <ros/ros.h>

#include <inspector_gcs/API_UavList.h>
#include <inspector_gcs/API_MissionCommand.h>


class RostfulServices {

    public:
        RostfulServices(ros::NodeHandle &_nh, std::vector<std::string> &_uav_list);
        // ~RostfulServices();
        
        


    private:
        // Ros Services
        ros::ServiceServer uav_list_server;
        ros::ServiceServer start_mission_server;
        ros::ServiceServer stop_mission_server;
        ros::ServiceServer resume_mission_server;

        bool uav_list_server_cb(inspector_gcs::API_UavList::Request &req, inspector_gcs::API_UavList::Response &res);
        bool start_mission_server_cb(inspector_gcs::API_MissionCommand::Request &req, inspector_gcs::API_MissionCommand::Response &res);
        bool stop_mission_server_cb(inspector_gcs::API_MissionCommand::Request &req, inspector_gcs::API_MissionCommand::Response &res);
        bool resume_mission_server_cb(inspector_gcs::API_MissionCommand::Request &req, inspector_gcs::API_MissionCommand::Response &res);
        
        // std::vector* myvector_ptr;
        std::vector<std::string>* _uav_list_ptr;
        // std::vector<std::string> uav_list;
};




#endif // ROSTFUL_SERVICES