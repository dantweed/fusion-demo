//
// Camera.h
// Extemds Device.cpp to add object tracking data
//
// Created by Daniel.Tweed on 11/2/2017.
//

#ifndef WRNCH_CAMERA_H
#define WRNCH_CAMERA_H

#include "Device.h"
#include <array>

//Extends Device to add tracking info reporting
class Camera : public Device {

private:
    std::array<double, 3> trackingData; //Location of tracked object in Camera's reference frame
    bool inField;       // Flag to indicate if X is in camera's field of view

public:
    Camera();
    std::array<double, 3> getTrackingData();
    void setTrackingData(std::array<double, 3>);
    void setInfield(bool);
    bool isInField();
};


#endif //WRNCH_CAMERA_H
