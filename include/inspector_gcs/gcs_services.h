#ifndef GCS_SERVICES_H
#define GCS_SERVICES_H

#include <ros/ros.h>
#include <inspector_gcs/UAV_Link.h>

class GcsServices {

    public:
        GcsServices(ros::NodeHandle &_nh, std::vector<std::string> &_uav_list);


    private:
        
        // ROS Services
        ros::ServiceServer uav_link_server;

        bool uav_link_server_cb(inspector_gcs::UAV_Link::Request &req, inspector_gcs::UAV_Link::Response &res);

        std::vector<std::string>* _uav_list_ptr;


};

#endif // GCS_SERVICES_H