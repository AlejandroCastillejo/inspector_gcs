#include <iostream>
#include <ros/ros.h>
#include <inspector_gcs/gcs_services.h>
#include <inspector_gcs/UavList.h>

// UAV list
std::vector<std::string> uav_list;
// std::vector<std::string> uav_list = {"uav_1", "uav_2", "uav_3", "uav_n"};

// ROS 
ros::Publisher uav_list_pub;
inspector_gcs::UavList uav_list_msg;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "inspector_gcs");
  ros::NodeHandle nh;
  GcsServices gcs_services(nh, uav_list);
  ROS_INFO("inspector_gcs running");
    
  uav_list_pub = nh.advertise<inspector_gcs::UavList>("uav_list", 1);   // uav list publisher

  while(ros::ok()){
    uav_list_msg.uavs = uav_list;
    uav_list_pub.publish(uav_list_msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
}