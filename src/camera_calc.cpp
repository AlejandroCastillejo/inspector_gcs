#include <inspector_gcs/camera_calc.h>
#include <ros/package.h>


CameraCalc::CameraCalc(std::string _camera_name) {

  std::string package_path = ros::package::getPath("inspector_gcs");
  std::string cameras_file_location = package_path + "/json_files/cameras.json";
  // std::cout << "cameras_file_location" << cameras_file_location << std::endl;

  cameras_js = json::array({});
  // std::ifstream file("cameras.json");
  std::ifstream file(cameras_file_location);
  file >> cameras_js;

    // std::cout << cameras_js["cameras"] << std::endl;
  spec.horizontalAOV = cameras_js["cameras"][_camera_name.c_str()]["horizontalAOV"];
  spec.verticalAOV = cameras_js["cameras"][_camera_name.c_str()]["verticalAOV"];
};

double CameraCalc::horizontalFOV(double _height) {
    return 2*_height*tan(spec.horizontalAOV*PI/180/2);
};

double CameraCalc::verticalFOV(double _height) {
    return 2*_height*tan(spec.verticalAOV*PI/180/2);
};
