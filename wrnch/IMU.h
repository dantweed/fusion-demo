//
// IMU.h
// 9DOF IMU with accelerometers, gyroscopes, and magnetometers
//  all independent 3-axis.
//
// Created by Daniel.Tweed on 11/1/2017.
//

#ifndef WRNCH_IMU_H
#define WRNCH_IMU_H

#include <array>
#include "etk/etk.h"

class IMU {
private:
    std::array<double,3> acc;
    std::array<double,3> gyro;
    std::array<double,3> mag;

public:
    IMU();

    std::array<double,3> getAcc();
    std::array<double,3> getGyro();
    std::array<double,3> getMag();

    void setAcc(std::array<double,3>);
    void setGyro(std::array<double,3>);
    void setMag(std::array<double,3>);
};

#endif //WRNCH_IMU_H
