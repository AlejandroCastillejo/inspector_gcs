#include <iostream>
#include <list>
#include <string>
#include <thread>
#include <cmath>

#include <ros/ros.h>

// ROS Services & Msgs
#include <inspector_gcs/gcsCreateMission.h>
#include <inspector_gcs/gcsSendMission.h>
#include <inspector_gcs/MissionService.h>
#include <uav_abstraction_layer/State.h>

#include <visualization_msgs/Marker.h>

// #include "qt-GCS/mainwindow.h"
#include <QApplication>
#include <QtDebug>
// #include <QList>

#include <parametros.h>
#include <inspector_gcs/get_from_json.h>
#include <inspector_gcs/mission_builder.h>
#include <inspector_gcs/rostful_services.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


typedef uav_abstraction_layer::State ualState;

struct uav_state {
  bool connected;
  ros::Time last_msg;
  ualState ual_state;
  std::string adl_state;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped velocity;
}uav_state_empty;

std::map<std::string, uav_state> map_uav_state;
std::map<std::string, uav_state>::iterator iter_uav;
// UAV list
std::vector<std::string> uav_list = {"uav_1", "uav_2", "uav_3", "uav_n"};
// std::vector<std::string> uav_list = {"uav_1"};
// std::vector<std::string> uav_list;

//Global variables
ros::NodeHandle *p_nh;
QGeoCoordinate BaseCoordinate;
QList<QGeoCoordinate> PolygonCoordinates;
double h_c; //flight altitude

// QList<QGeoCoordinate> WaiPoints;
QList<QLineF> resultLinesNED;
QList<QList<QGeoCoordinate>> resultTransectsGeo;
QList<QList<QPointF>> droneWayPointsNED;
QList<QList<QGeoCoordinate>> droneWayPointsGeo;
QString qfilelocation;
std::vector<nav_msgs::Path> missionPaths;
std::vector<geographic_msgs::GeoPath> missionPathsGeo;
void visualization(void);
// bool adv_mission(inspector_gcs::MissionService::Request  &req, inspector_gcs::MissionService::Response &res);
bool mission_created = false;

// Thread
std::thread main_thread;
void mainThread();

// ROS subscribers CallBAcks
void uav_state_sub_cb(const uav_abstraction_layer::State::ConstPtr& msg, const std::string _uav_id);

// ROS Service CallBacks
bool create_mission_cb(inspector_gcs::gcsCreateMission::Request &req, inspector_gcs::gcsCreateMission::Response &res);
bool send_mission_cb(inspector_gcs::gcsSendMission::Request &req, inspector_gcs::gcsSendMission::Response &res);

// ROS Publishers
ros::Publisher path_pub;
ros::Publisher geo_path_pub;


GetFromJson g_Json;
MissionBuilder mb;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "inspector_gcs");
  ros::NodeHandle nh;
  p_nh = &nh; //pointer to nh

  RostfulServices rostful(nh, uav_list);


  // print list of UAVs
  std::cout << "list of UAVs:" << std::endl;
  for (auto uav_id : uav_list) {
    std::cout << "  " << uav_id << std::endl;
  }
  n = uav_list.size();
  std::cout << "\n" << "uav_list size: " << n << "\n" << std::endl;
  
  // Rostful Services
  // rostful.uav_list_server = nh.advertiseService("uav_list", &RostfulServices::uav_list_server_cb, this);
  
  // RostfulServices* rosful_ptr = &rostful;
  // rostful.uav_list_server = nh.advertiseService("uav_list", &RostfulServices::uav_list_server_cb, rosful_ptr);

  // std::vector myvector = {0, 0,1};
  // RostfulServices myobject(&myvector);

  
  
  // Subscribers
  ros::Subscriber uav_state_sub[n];

  // for (auto uav_id : uav_list) {
  for (int i=0; i<uav_list.size(); i++) {
    std::string uav_id = uav_list[i];
    std::cout << "subscribed to " << uav_id << "/state" << std::endl;
    uav_state_sub[i] = nh.subscribe<ualState>(uav_id + "/ual/state", 1, std::bind(uav_state_sub_cb, std::placeholders::_1, uav_id));

    map_uav_state.insert(std::pair<std::string, uav_state>(uav_list[i], uav_state_empty));
    // map_uav_state.insert((uav_list[1]));
  }
  // for (iter_uav = map_uav_state.begin(); iter_uav != map_uav_state.end(); ++iter_uav) {
  //   uav_state_sub[n] = nh.subscribe<ualState>(uav_id +"/ual/state", 1, std::bind(uav_state_sub_cb, std::placeholders::_1, uav_id));
  // }
    
  // GCS Services
  ros::ServiceServer create_mission_srv = nh.advertiseService("create_mission_service", create_mission_cb);
  ros::ServiceServer send_mission_srv = nh.advertiseService("send_mission_service", send_mission_cb);

  main_thread = std::thread(mainThread);

  path_pub = nh.advertise<nav_msgs::Path>("path", 100);
  geo_path_pub = nh.advertise<geographic_msgs::GeoPath>("geo_path", 100);
 
  while(ros::ok()){


  // visualization();
    ros::spinOnce();
  }
}

void mainThread() {
  while(ros::ok()) {
    for (iter_uav = map_uav_state.begin(); iter_uav != map_uav_state.end(); ++iter_uav) {
      if (ros::Time::now().toSec() - iter_uav->second.last_msg.toSec() > 0.5) {
        iter_uav->second.connected = false;
        // ROS_INFO("Connection with %s lost", iter_uav->first.c_str());
      }
    }
    // for (iter_uav = map_uav_state.begin(); iter_uav != map_uav_state.end(); ++iter_uav) {
    //   if (ros::Time::now().toSec() - iter_uav->second.last_msg.toSec() > 0.5) {
    //     iter_uav->second.connected = false;
    //     ROS_INFO("Connection with %s lost", iter_uav->first.c_str());
    //   }
    // }
    ros::Duration(0.1).sleep();
  }
}


// Services callbacks definitions //

bool create_mission_cb(inspector_gcs::gcsCreateMission::Request &req, inspector_gcs::gcsCreateMission::Response &res) {
  ROS_INFO("Creating mission");
  // GET PERIMETER & PARAMETERS FROM JSON FILE
  g_Json.GetCoordinates(PolygonCoordinates, h_c, qfilelocation);  
  //
  // Test
    std::cout << "\nFlight height: " << h_c << std::endl;
    std::cout << "\nPolygonCoordinates \n" << std::endl;
    // std::list<QGeoCoordinate>::iterator iter;
    QList<QGeoCoordinate>::iterator iter;
    for( iter = PolygonCoordinates.begin(); iter != PolygonCoordinates.end(); iter++) {
      // std::cout.precision(10);
       std::cout << iter->latitude() << "    " <<iter->longitude() << std::endl;
    };

  BaseCoordinate = QGeoCoordinate{37.558632, -5.931218};

  //
  // OBTAIN MISSION WAYPOINTS
  mb._buildTransects(BaseCoordinate, PolygonCoordinates, resultLinesNED, resultTransectsGeo);
  mb._buildMission(n, resultLinesNED, droneWayPointsNED);
  // qDebug() << "drone waypoints" << droneWayPointsNED;

  // CONVERT WAYPOINTS TO GEOCOORDINATES
  QGeoCoordinate tangentOrigin = PolygonCoordinates[0];
  QList<QGeoCoordinate> idroneWayPointsGeo;
  QGeoCoordinate coord;
  qDebug() << "tangent origin" << tangentOrigin << endl;
  // qDebug() << droneWayPointsNED << endl;
  for (int i=0; i<n; i++) {
  qDebug() << endl;
  // qDebug() << i;
    for (int j=0; j<droneWayPointsNED[i].size(); j++) {
      // qDebug() << j;
      // qDebug() << droneWayPointsNED[i][j];
      convertNedToGeo(droneWayPointsNED[i][j].x(), droneWayPointsNED[i][j].y(), 0, tangentOrigin, coord);
      // qDebug() << coord;
      idroneWayPointsGeo.append(coord);
    }
    droneWayPointsGeo.append(idroneWayPointsGeo);
    idroneWayPointsGeo.clear();

    qDebug() << "drone"<< i << "WayPoints NED\n" << droneWayPointsNED << endl;
    qDebug() << "drone"<< i << "WayPoints Geo\n" << droneWayPointsGeo << endl;
  }
  // OBTAIN SAFE POSITIONING HEIGHTs
  mb._heightDistribution(n, h_d, h_c, h_min, d);

  //CREATE MISSION MESSAGES
  missionPaths = mb._createMissionPaths(droneWayPointsNED, h_d, h_c);
  missionPathsGeo = mb._createMissionPathsGeo(droneWayPointsGeo, h_d, h_c);
  
  qDebug()<<endl;
  qDebug()<< "drone waypoints geo" << endl << droneWayPointsGeo << endl;
  qDebug()<< "drone waypoints NED" << endl << droneWayPointsNED << endl;
  // qDebug()<< "missionPaths"<< missionPaths << endl;
  // qDebug()<< "missionPathsGeo"<< missionPathsGeo << endl;
  ROS_INFO("Ready to send mission \n");
  mission_created = true;
  res.result = true;
  // visualization();
    
  path_pub.publish(missionPaths[0]);
  geo_path_pub.publish(missionPathsGeo[0]);
  
  return true;
}

bool send_mission_cb(inspector_gcs::gcsSendMission::Request &req, inspector_gcs::gcsSendMission::Response &res) {
  if (!mission_created) {
    ROS_ERROR("Not mission available. Please create mission");
    return false;
  }
  ROS_INFO("Sending mission");

  // ros::NodeHandle nh;
  // GENERATE MISSIONS SERVICES
  n = uav_list.size();
  ros::ServiceClient mission_client[n];
  inspector_gcs::MissionService mission_srv[n];
  // std::string mission_service_i[n];

  for (int i=0; i<n; i++) {
      // ros::ServiceClient client = nh.serviceClient<inspector_gcs::MissionService>("mission_service_%d", i);
      // std::string ms_string = "mission_service_%d", i;
      // mission_client[i] = p_nh->serviceClient<inspector_gcs::MissionService>("mission_service_" + std::to_string(i));
      mission_client[i] = p_nh->serviceClient<inspector_gcs::MissionService>(uav_list[i] + "/mission_service");

      mission_srv[i].request.h_d = h_d[i];
      // mission_srv[i].request.MissionPath = missionPaths[i];
      mission_srv[i].request.MissionPath = missionPathsGeo[i];
      if (mission_client[i].call(mission_srv[i])) {
        ROS_INFO("Mission sent to %s", uav_list[i].c_str());
      }
      else {
        ROS_ERROR("Failed to call %s/mission_sevice", uav_list[i].c_str());
        // return 1;
      }
    }

  res.result = true;
  return true;
}

// Subscribers callbacks

void uav_state_sub_cb(const uav_abstraction_layer::State::ConstPtr& msg, const std::string _uav_id) {
  // ROS_INFO("%s connected", _uav_id.c_str());

  map_uav_state["_uav_id"].connected = true;
  // ROS_INFO("%s connected", iter_uav->first.c_str());
  map_uav_state["_uav_id"].last_msg = ros::Time::now();
  map_uav_state["_uav_id"].ual_state = *msg;

}


// bool adv_mission(inspector_gcs::MissionService::Request  &req, inspector_gcs::MissionService::Response &res)
//   {
//     if(h_d.size() == missionPaths.size()) {
//       ROS_INFO("test1");
//       res.MissionPaths = missionPaths;
//       ROS_INFO("test2");
//       for (int h : h_d) {
//         res.H_d.push_back(h);
//         // ROS_INFO("[%d]",h);

//       }
//       ROS_INFO("test3");
//       res.h2 = 2;
//       // int h3;
//       // h3 = res.h2;
//       // for(int h : h2) {
//       //   qDebug()<< h;    
//       // } 
//       // res.H_d = h_d;
//       ROS_INFO(" [%d]",res.h2);
//       return true;
//     }
//     else {
//       return false;
//     }
    
//   }

  //////////      ////////
  // ROS VISUALIZATION //
  //////////////////////
  void visualization()
  {
    ros::NodeHandle nh;
    ros::Publisher _pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate r(1);

    while (ros::ok()) {
     
     visualization_msgs::Marker points, line_strip, line_strip2;
     points.header.frame_id = line_strip.header.frame_id = "/inspector_gcs";
     points.header.stamp = line_strip.header.stamp = ros::Time::now();
     points.ns = line_strip.ns = "points";
     points.action =  line_strip.action = visualization_msgs::Marker::ADD;
     points.pose.orientation.w =line_strip.pose.orientation.w = 1.0;
     
     points.id = 0;
     line_strip.id = 1;

     points.type = visualization_msgs::Marker::POINTS;
     line_strip.type = visualization_msgs::Marker::LINE_STRIP;

     // POINTS markers use x and y scale for width/height respectively
      points.scale.x = 0.2;
      points.scale.y = 0.2;
     
     // Points are green
     points.color.g = 1.0f;
     points.color.a = 1.0;

     // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip.scale.x = 0.1;

        // Line strip is blue
      line_strip.color.b = 1.0;
      line_strip.color.a = 1.0;
   
     for (int i=0; i<droneWayPointsNED.size(); i++) {
       for (int j=0; j<droneWayPointsNED[i].size(); j++) {
        geometry_msgs::Point p;
        p.x = droneWayPointsNED[i][j].x();
        p.y = droneWayPointsNED[i][j].y();
        p.z = 2;

        points.points.push_back(p);
        line_strip.points.push_back(p);
          
      // qDebug()<< p.x << p.y << p.z;
       }        
     }
     _pub.publish(points);
     _pub.publish(line_strip);

     sleep(1); 
     }
  }
