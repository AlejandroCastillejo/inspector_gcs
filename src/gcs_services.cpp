#include <inspector_gcs/gcs_services.h>
#include <string>

GcsServices::GcsServices(ros::NodeHandle &_nh, std::vector<std::string> &_uav_list)
{


    uav_link_server = _nh.advertiseService("uav_link", &GcsServices::uav_link_server_cb, this);

}

bool GcsServices::uav_link_server_cb(inspector_gcs::UAV_Link::Request &req, inspector_gcs::UAV_Link::Response &res)
{
    ROS_INFO("%s link request", req.uav_id.c_str());

    std::vector<std::string> uav_list = *_uav_list_ptr;
    
    std::vector<std::string>::iterator it;
    it = find (uav_list.begin(), uav_list.end(), req.uav_id);
    
    if (it != uav_list.end()) {
        ROS_WARN("%s was already linked", req.uav_id.c_str());
    }
    else {
        uav_list.push_back(req.uav_id);
        ROS_INFO("%s linked successfully", req.uav_id.c_str());
    }
    res.linked = true;

    return true;
}
