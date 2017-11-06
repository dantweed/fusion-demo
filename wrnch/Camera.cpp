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

/** Update current estimated position of X in camera field of view
 *
 * @param position in 3d array (p_x,p_y,p_z)
 */
void Camera::setTrackingData(std::array<double, 3> newTrack){
    trackingData = newTrack;
    setInfield(true);
}

/** Set flag indicating if object is in camera field of view
 *
 * @param new inField value
 */
void Camera::setInfield(bool val) {
    inField = val;
}

/** Acces flag indicating if object is in camera field of view
 *
 * @return In field flag
 */
bool Camera::isInField(){
    return inField;
}
