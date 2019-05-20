#include <inspector_gcs/rostful_services.h>

#include <ros/ros.h>



// RostfulServices::RostfulServices(std::vector &myvector)
RostfulServices::RostfulServices(ros::NodeHandle &_nh, std::vector<std::string> &_uav_list)
{

    // ros::NodeHandle nh;
    // myvector_ptr = myvector;
    _uav_list_ptr = &_uav_list;
    // uav_list = _uav_list;

    // uav_list_server = nh.advertiseService("uav_list", uav_list_server_cb);
     uav_list_server = _nh.advertiseService("uav_list", &RostfulServices::uav_list_server_cb, this);
     start_mission_server = _nh.advertiseService("start_mission", &RostfulServices::start_mission_server_cb, this);
     stop_mission_server = _nh.advertiseService("stop_mission", &RostfulServices::stop_mission_server_cb, this);
     resume_mission_server = _nh.advertiseService("resume_mission", &RostfulServices::resume_mission_server_cb, this);


}

bool RostfulServices::uav_list_server_cb(inspector_gcs::API_UavList::Request &req, inspector_gcs::API_UavList::Response &res)
{   
    std::vector<std::string> uav_list = *_uav_list_ptr;
    for (int i=0; i<uav_list.size(); i++) {
        std::string uav_id = uav_list[i] + ", ";
        // std::string uav_id = uav_list[i] + ", ";
        // res.uav_list[i] = uav_id.c_str();
        res.uav_list.append(uav_id.c_str());
    }
    return true;
}

bool RostfulServices::start_mission_server_cb(inspector_gcs::API_MissionCommand::Request &req, inspector_gcs::API_MissionCommand::Response &res) 
{
    std::cout << req.uav << std::endl;
    return true;
}

bool RostfulServices::stop_mission_server_cb(inspector_gcs::API_MissionCommand::Request &req, inspector_gcs::API_MissionCommand::Response &res) 
{
    std::cout << req.uav << std::endl;
    return true;
}

bool RostfulServices::resume_mission_server_cb(inspector_gcs::API_MissionCommand::Request &req, inspector_gcs::API_MissionCommand::Response &res) 
{
    std::cout << req.uav << std::endl;
    return true;
}

// RostfulServices rost, rost1;