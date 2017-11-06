//
// Camera.cpp
// Extemds Device.cpp to add object tracking data
//
// Created by Daniel.Tweed on 11/2/2017.
//

#include "Camera.h"

//Default constructor
Camera::Camera() {
    trackingData = {0,0,0}; //Initialize to zero and assume unknown
    inField = false;        //as initially not in field of view
}

/** Access and return current estimated position of X in camera field of view
 *
 * @return position in 3d array (p_x,p_y,p_z)
 */
std::array<double, 3> Camera::getTrackingData(){
    std::array<double, 3> rtnVal = trackingData;
    return rtnVal;
}
void Camera::setTrackingData(std::array<double, 3> newTrack){
    trackingData = newTrack;
    setInfield(true);
}



void Camera::setInfield(bool val) {
    inField = val;
}

bool Camera::isInField(){
    return inField;
}
