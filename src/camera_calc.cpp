#include <inspector_gcs/camera_calc.h>


CameraCalc::CameraCalc(std::string _camera_name) {
  
  cameras_js = json::array({});
  std::ifstream file("cameras.json");
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
