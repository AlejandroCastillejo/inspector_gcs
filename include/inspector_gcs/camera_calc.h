#ifndef CAMERA_CALC_H
#define CAMERA_CALC_H

// #include <ros/ros.h>
#include <json.hpp> // single-include file for json parsing
#include <fstream>
#include <iostream>

#define PI 3.14159265

using json = nlohmann::json;

struct CameraSpec {
    // double sensorWidth;
    // double sensorHeight;
    // double focalLenght;
    double horizontalAOV;
    double verticalAOV;
    double AngleOfView;
    double horizontalResolution;
    double verticalResolution;
};


class CameraCalc {

    public:
        CameraCalc(std::string _camera_name);
        double horizontalFOV(double _height);
        double verticalFOV(double _height);
    
    private:
        CameraSpec spec;
        json cameras_js;


};

#endif //CAMERA_CALC_H