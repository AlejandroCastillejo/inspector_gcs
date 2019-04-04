#include <global_tracker/global_tracker.h>
#include <functional>
#include <geometry_msgs/Point32.h>
#include <multidrone_kml_parser/geographic_to_utm_to_cartesian.hpp>

namespace multidrone
{

// Brief Constructor
GlobalTracker::GlobalTracker()
{

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Read parameters
    // drones [id_1,...,id_N]
    pn.getParam("drones", drones_vector_);

    // Subscribers
    for (auto drone_id : drones_vector_)
    {
        drone_target_sub_[drones_[i]] = n.subscribe<multidrone_msgs::TargetStateArray>("drone_" + std::to_string(drones_[i]) + "/target_3d_pose", 1, 
                                        std::bind(&GlobalTracker::targetDroneCallback, this, std::placeholders::_1, drones_[i])); // Change for each drone ID
    }

    // Make communications spin!
    spin_thread_ = std::thread( [this]() {
        ros::MultiThreadedSpinner spinner(2); // Use 2 threads
        spinner.spin();
    });

/// Main loop
mainloop_thread_ = std::thread([this]() {
    // Publish @ 10Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
    // Comprobar Time::now() con cada last_msg

    loop_rate.sleep();
    }
});

}

// Brief Destructor
GlobalTracker::~GlobalTracker()
{
}


void GlobalTracker::ualStatusCallback(const grvc::ual::Status::ConstPtr& _msg, const int _drone_id)
{
drone_list[drone_id].ual_status = _msg->status;
drone_list[drone_id].connected =true;
drone_list[drone_id].last_msg = ros::Time::now();
}