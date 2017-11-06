//
// DeviceState.h
// Calculate and present the properties of a device in 3-D space, relative to its own reference frame
// from IMU sensor data.
//   velocity, position, orientation.
// Created by Daniel.Tweed on 11/2/2017.
//

#ifndef WRNCH_DEVICESTATE_H
#define WRNCH_DEVICESTATE_H

#include "Device.h"
#include <array>
#include "etk/etk.h"
#include "MDKalman.h"
#include "ArrayArithmetic.h"

class DeviceState {
public:
    DeviceState() = delete;
    DeviceState(Device &,float);
    static const double G; //Gravitational acceleration m/s^2

    std::array<double,3> getEstVel();
    std::array<double,3> getEstPos();    
    std::array<double,3> getEstOrientAngles();
    etk::Quaternion getEstOrientQ();

    void updateFromIMU(std::array<std::array<double,3>,3> imuData);

private:
    Device & dev;
    float deltaT;   //Device data update period

    //Relative to own reference frame
    std::array<double,3> estPos;
    std::array<double,3> estVel;    // m/s
    etk::Quaternion estOrient;      // Quaternion represntation of orientation

    //Kalman filters for each of the sensor axes
    MDKalman<3> acc;
    MDKalman<3> gyro;
    MDKalman<3> mag;

    void updateVel();
    void updatePos();
    void updateOrient();

};


#endif //WRNCH_DEVICESTATE_H
