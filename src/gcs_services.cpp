#include <inspector_gcs/gcs_services.h>
#include <string>

std::string mission_file_location = ros::package::getPath("inspector_gcs") + "/json_files/mission_file.json";

GcsServices::GcsServices(ros::NodeHandle &_nh, std::vector<std::string> &_uav_list, QList<QList<QPointF>> &_droneWayPointsNED) 
  : get_from_json(mission_file_location), 
    mission_builder(), 
    rgb_camera(rgb_camera_name), 
    thermal_camera(thermal_camera_name)
{
    
    p_nh = &_nh;                //pointer to nh
    p_uav_list = &_uav_list;    

    // package_path = ros::package::getPath("inspector_gcs");
    // mission_file_location = package_path + "/json_files/mission_file.json";


    // get_from_json = GetFromJson::(mission_file_location);
    // MissionBuilder mission_builder;
    // CameraCalc Objects
    // CameraCalc() : rgb_camera("Sony a6000");
    // rgb_camera = CameraCalc::("Sony a6000");
    // CameraCalc thermal_camera("WIRIS 2nd gen");

    uav_link_srv = _nh.advertiseService("uav_link_service", &GcsServices::uav_link_server_cb, this);
    create_mission_srv = _nh.advertiseService("create_mission_service", &GcsServices::create_mission_cb, this);
    send_mission_srv = _nh.advertiseService("send_mission_service", &GcsServices::send_mission_cb, this);

}

bool GcsServices::uav_link_server_cb(inspector_gcs::uavLink::Request &req, inspector_gcs::uavLink::Response &res)
{
    std::string uav_id = req.uav_id;
    ROS_INFO("%s link request", uav_id.c_str());
    
    std::vector<std::string>::iterator it = find (p_uav_list->begin(), p_uav_list->end(), uav_id);
    if (it != p_uav_list->end()) {
        ROS_WARN("%s was already linked", uav_id.c_str());
    }
    else {
        p_uav_list->push_back(uav_id);
        ROS_INFO("%s linked successfully", uav_id.c_str());
        res.linked = true;
    }
    
    // update uav_list & n
    uav_list = *p_uav_list;
    n = uav_list.size();
    
    // print current uav list
    std::cout << "uav_list: " << std::endl;
    for (int i=0; i<uav_list.size(); i++) {
        std::cout << uav_list[i] << std::endl;
    }
    return true;
}

bool GcsServices::create_mission_cb(inspector_gcs::gcsCreateMission::Request &req, inspector_gcs::gcsCreateMission::Response &res) 
{
  ROS_INFO("Creating mission");

  // clear lists
  h_d.clear();
  resultLinesNED.clear();
  resultTransectsGeo.clear();
  droneWayPointsNED.clear();
  droneWayPointsGeo.clear();
 
  // print uav list & size
  std::cout << "list of UAVs:" << std::endl;
  for (auto uav_id : uav_list) {
    std::cout << "  " << uav_id << std::endl;
  }
  std::cout << "\n" << "uav_list size: " << n << "\n" << std::endl;
  
  // GET PERIMETER & PARAMETERS FROM JSON FILE
  get_from_json.GetCoordinates(PolygonCoordinates);
  // gridAngle = get_from_json.GetFlightDirection() - 90;
  gridAngle = get_from_json.GetFlightDirection();
  h_c = get_from_json.GetFlightAltitude();
  //
  // Test
    std::cout << "\nFlight height: " << h_c << std::endl;
    std::cout << "\nPolygonCoordinates \n" << std::endl;
    QList<QGeoCoordinate>::iterator iter;
    for( iter = PolygonCoordinates.begin(); iter != PolygonCoordinates.end(); iter++) {
      // std::cout.precision(10);
       std::cout << iter->latitude() << "    " <<iter->longitude() << std::endl;
    };

  // Obtain Base Coordinate from first UAV in uav_list
  // uav_pose_sub = p_nh->subscribe<geometry_msgs::PoseStamped>(uav_list[0] + "/ual/pose", 1, uav_pose_sub_cb);
  uav1_coordinate_sub = p_nh->subscribe<sensor_msgs::NavSatFix>(uav_list[0] + "/dji_sdk/gps_position", 1, \
        [this](const sensor_msgs::NavSatFix::ConstPtr& _msg) {
            this->uav1_coordinate = *_msg;
            this->BaseCoordinate = QGeoCoordinate{uav1_coordinate.latitude, uav1_coordinate.longitude};
            qDebug() << "BaseCoordinate: " << BaseCoordinate << endl;

  });
  // while (!BaseCoordinate.isValid()) {
  //   std::cout << "waiting for BaseCoordinate to be set" << std::endl;
  //   ros::Duration(0.5).sleep();
  // }
  std::cout << uav1_coordinate << std::endl;
  uav1_coordinate_sub.shutdown();
    // test
  BaseCoordinate = QGeoCoordinate{37.091142, -5.872402};    // manually set Base Coordinate
  // BaseCoordinate = QGeoCoordinate{37.356516, -6.126977};
  qDebug() << "BaseCoordinate: " << BaseCoordinate << endl;


  std::cout << "rgb_camera horizontalFOV: " << rgb_camera.horizontalFOV(h_c) << std::endl;
  
  // OBTAIN gidSpacing
  double min_verticalFOV = std::min(rgb_camera.verticalFOV(h_c), thermal_camera.verticalFOV(h_c) );
  std::cout << "min verticalFOV: "<< min_verticalFOV << std::endl;
  double transv_overlap_ideal = get_from_json.GetTransverseOverlapIdeal();
  std::cout << "ideal transverse overlap: "<< transv_overlap_ideal << std::endl;
  gridSpacing = min_verticalFOV * (1.0 - transv_overlap_ideal);
  std::cout << "gridSpacing: "<< gridSpacing << std::endl;

  //
  // OBTAIN MISSION WAYPOINTS
  mission_builder._buildTransects(BaseCoordinate, PolygonCoordinates, gridAngle, gridSpacing, resultLinesNED, resultTransectsGeo);
  mission_builder._buildMission(n, resultLinesNED, droneWayPointsNED);
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
  mission_builder._heightDistribution(n, h_d, h_c, h_min, d);

  //CREATE MISSION MESSAGES
  missionPaths = mission_builder._createMissionPaths(droneWayPointsNED, h_d, h_c);
  missionPathsGeo = mission_builder._createMissionPathsGeo(droneWayPointsGeo, h_d, h_c);
  
  qDebug()<<endl;
  qDebug()<< "drone waypoints geo" << endl << droneWayPointsGeo << endl;
  qDebug()<< "drone waypoints NED" << endl << droneWayPointsNED << endl;
  // qDebug()<< "missionPaths"<< missionPaths << endl;
  // qDebug()<< "missionPathsGeo"<< missionPathsGeo << endl;
  ROS_INFO("Ready to send mission \n");
 
 if (!mission_created) {
    vis_thread = std::thread(&GcsServices::visualization_thread, this, droneWayPointsNED, droneWayPointsGeo, h_d, h_c);
 }

  mission_created = true;
  res.result = true;
  return true;
}

bool GcsServices::send_mission_cb(inspector_gcs::gcsSendMission::Request &req, inspector_gcs::gcsSendMission::Response &res) 
{
  if (!mission_created) {
    ROS_ERROR("Not mission available. Please create mission");
    return false;
  }
  ROS_INFO("Sending mission");

  // GENERATE MISSIONS SERVICES
  n = uav_list.size();
  ros::ServiceClient mission_client[n];
  inspector_gcs::MissionService mission_service[n];
  //
  // GET FLIGHT ANGLE
  double flight_angle = get_from_json.GetFlightDirection();
  double orientation;
  orientation = flight_angle - 90;
  //
  // CALCULATE SHOOTING DISTANCE
  double thermal_camera_horizontalFOV = thermal_camera.horizontalFOV(h_c);
  double rgb_camera_horizontalFOV = rgb_camera.horizontalFOV(h_c);
  double ideal_long_overlap = get_from_json.GetLongitudinalOverlapIdeal();
  double thermal_camera_shoot_dist = thermal_camera_horizontalFOV * (1.0 - ideal_long_overlap);
  double rgb_camera_shoot_dist = rgb_camera_horizontalFOV * (1.0 - ideal_long_overlap);
  
  std::cout << "thermal_camera_shoot_dist: " << thermal_camera_shoot_dist << std::endl;
  std::cout << "rgb_camera_shoot_dist: " << rgb_camera_shoot_dist << std::endl;
  //

  for (int i=0; i<n; i++) {
      // ros::ServiceClient client = nh.serviceClient<inspector_gcs::MissionService>("mission_service_%d", i);
      // std::string ms_string = "mission_service_%d", i;
      // mission_client[i] = p_nh->serviceClient<inspector_gcs::MissionService>("mission_service_" + std::to_string(i));
      mission_client[i] = p_nh->serviceClient<inspector_gcs::MissionService>(uav_list[i] + "/mission_service");

      mission_service[i].request.h_d = h_d[i];
      mission_service[i].request.flight_angle = flight_angle;
      mission_service[i].request.orientation = orientation;
      mission_service[i].request.thermal_camera_shooting_distance = thermal_camera_shoot_dist;
      mission_service[i].request.rgb_camera_shooting_distance = rgb_camera_shoot_dist;
      // mission_service[i].request.MissionPath = missionPaths[i];
      mission_service[i].request.MissionPath = missionPathsGeo[i];
      if (mission_client[i].call(mission_service[i])) {
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

void GcsServices::visualization_thread(QList<QList<QPointF>> _droneWayPointsNED, QList<QList<QGeoCoordinate>> _droneWayPointsGeo,  std::vector<int> _h_d, int _h_c) 
{
  // publish rviz_satellite reference
  sensor_msgs::NavSatFix map_origin;
  ros::Publisher map_origin_pub = p_nh->advertise<sensor_msgs::NavSatFix>("/gps/fix", 1);

  map_origin.header.seq = 999;
  map_origin.header.stamp = ros::Time::now();
  map_origin.header.frame_id = "map";
  map_origin.status.status = 0;
  map_origin.status.service = 1;
  map_origin.latitude = PolygonCoordinates[0].latitude();
  map_origin.longitude = PolygonCoordinates[0].longitude();
  map_origin.altitude = 0;
  map_origin.position_covariance = {3.9561210000000004, 0.0, 0.0, 0.0, 3.9561210000000004, 0.0, 0.0, 0.0, 7.650756};
  map_origin.position_covariance_type = 2;

  map_origin_pub.publish(map_origin);
  // 

  int n = _droneWayPointsNED.size();

  // ros::Publisher _pub = p_nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher _pub[10];
  for (int i = 0; i < 10; i++) {
    _pub[i] = p_nh->advertise<visualization_msgs::Marker>("visualization_marker_" + std::to_string(i), 10);
    // _pub[i] = p_nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }
  
  // define base local coordinate
  geometry_msgs::Point localBaseCoordinate;
  convertGeoToNed(BaseCoordinate, PolygonCoordinates[0], &localBaseCoordinate.x, &localBaseCoordinate.y, &localBaseCoordinate.z);
  std::cout << "localBaseCoordinate" << localBaseCoordinate << std::endl;

  while (ros::ok())
  {
    map_origin_pub.publish(map_origin);

    visualization_msgs::Marker points[n], line_strip[n];

    for (int i = 0; i < n; i++) {
      points[i].id = 0;
      line_strip[i].id = 1;
      points[i].type = visualization_msgs::Marker::POINTS;
      line_strip[i].type = visualization_msgs::Marker::LINE_STRIP;
      // POINTS markers use x and y scale for width/height respectively
      points[i].scale.x = 1.5;
      points[i].scale.y = 1.5;
      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip[i].scale.x = 1;

      // std::cout << "i%3  " << (i+3)%3 << std::endl;
      switch (i%3) {
        case 0:
          // Points are green
          points[i].color.r = 0.0;
          points[i].color.g = 1.0;
          points[i].color.b = 0.0;
          points[i].color.a = 1.0;
          // Line strip is green
          line_strip[i].color.r = 0.0;
          line_strip[i].color.g = 1.0;
          line_strip[i].color.b = 0.0;
          line_strip[i].color.a = 1.0;
          break;

        case 1:
          // Points are blue
          points[i].color.r = 0.0;
          points[i].color.g = 0.0;
          points[i].color.b = 1.0;
          points[i].color.a = 1.0;
          // Line strip is blue
          line_strip[i].color.r = 0.0;
          line_strip[i].color.g = 0.0;
          line_strip[i].color.b = 1.0;
          line_strip[i].color.a = 1.0;
          break;

        case 2:
          // Points are red
          points[i].color.r = 1.0;
          points[i].color.g = 0.0;
          points[i].color.b = 0.0;
          points[i].color.a = 1.0;
          // Line strip is red
          line_strip[i].color.r = 1.0;
          line_strip[i].color.b = 0.0;
          line_strip[i].color.g = 0.0;
          line_strip[i].color.a = 1.0;
          break;
      }
    }

    ros::Time ros_time = ros::Time::now();

    for (int i=0; i<_droneWayPointsNED.size(); i++) {
      points[i].header.frame_id = line_strip[i].header.frame_id = "/map";
      // points[i].header.stamp = line_strip[i].header.stamp = ros::Time::now();
      points[i].header.stamp = line_strip[i].header.stamp = ros_time;
      points[i].ns = line_strip[i].ns = "points";
      points[i].action =  line_strip[i].action = visualization_msgs::Marker::ADD;
      points[i].pose.orientation.w =line_strip[i].pose.orientation.w = 1.0;
      
      geometry_msgs::Point p;

      p.x = localBaseCoordinate.y;
      p.y = localBaseCoordinate.x;
      p.z = 0;
      points[i].points.push_back(p);
      line_strip[i].points.push_back(p);

      p.z = _h_d[i];
      points[i].points.push_back(p);
      line_strip[i].points.push_back(p);

      p.x = _droneWayPointsNED[i][0].y();
      p.y = _droneWayPointsNED[i][0].x();
      points[i].points.push_back(p);
      line_strip[i].points.push_back(p);

      for (int j=0; j<_droneWayPointsNED[i].size(); j++) {
        p.x = _droneWayPointsNED[i][j].y();
        p.y = _droneWayPointsNED[i][j].x();
        p.z = _h_c;  
        points[i].points.push_back(p);
        line_strip[i].points.push_back(p);
      }       
      
      int l = _droneWayPointsNED[i].size();
      p.x = _droneWayPointsNED[i][l-1].y();
      p.y = _droneWayPointsNED[i][l-1].x();
      p.z = _h_d[i];
      points[i].points.push_back(p);
      line_strip[i].points.push_back(p); 
      
      p.x = localBaseCoordinate.y;
      p.y = localBaseCoordinate.x;
      points[i].points.push_back(p);
      line_strip[i].points.push_back(p);
            
      p.z = 0;
      points[i].points.push_back(p);
      line_strip[i].points.push_back(p);

      _pub[i].publish(points[i]);
      _pub[i].publish(line_strip[i]);
      sleep(1); 
    }

    // for (int i=0; i<_droneWayPointsGeo.size(); i++) 
    // {
    //   points[i].header.frame_id = line_strip[i].header.frame_id = "/map";
    //   // points[i].header.stamp = line_strip[i].header.stamp = ros::Time::now();
    //   points[i].header.stamp = line_strip[i].header.stamp = ros_time;
    //   points[i].ns = line_strip[i].ns = "points";
    //   points[i].action =  line_strip[i].action = visualization_msgs::Marker::ADD;
    //   points[i].pose.orientation.w =line_strip[i].pose.orientation.w = 1.0;
      
    //   geometry_msgs::Point p;

    //   p.x = p.y = p.z = 0;
    //   points[i].points.push_back(p);
    //   line_strip[i].points.push_back(p);

    //   p.z = _h_d[i];
    //   points[i].points.push_back(p);
    //   line_strip[i].points.push_back(p);

    //   p.x = _droneWayPointsGeo[i][0].latitude();
    //   p.y = _droneWayPointsGeo[i][0].longitude();
    //   points[i].points.push_back(p);
    //   line_strip[i].points.push_back(p);

    //   for (int j=0; j<_droneWayPointsGeo[i].size(); j++) {
    //     p.x = _droneWayPointsGeo[i][j].latitude();
    //     p.y = _droneWayPointsGeo[i][j].longitude();
    //     p.z = _h_c;  
    //     points[i].points.push_back(p);
    //     line_strip[i].points.push_back(p);
    //   }       
      
    //   int l = _droneWayPointsGeo[i].size();
    //   p.x = _droneWayPointsGeo[i][l-1].latitude();
    //   p.y = _droneWayPointsGeo[i][l-1].longitude();
    //   p.z = _h_d[i];
    //   points[i].points.push_back(p);
    //   line_strip[i].points.push_back(p); 
      
    //   p.x = p.y = 0;
    //   points[i].points.push_back(p);
    //   line_strip[i].points.push_back(p);
            
    //   p.z = 0;
    //   points[i].points.push_back(p);
    //   line_strip[i].points.push_back(p);

    //   _pub[i].publish(points[i]);
    //   _pub[i].publish(line_strip[i]);
    //   sleep(1); 
    // }
  }  
}