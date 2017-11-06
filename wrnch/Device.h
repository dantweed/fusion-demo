//
// Device.h
// An arbitrary device with on-board 9DOF IMU
//
// Created by Daniel.Tweed on 11/2/2017.
//

#ifndef WRNCH_DEVICE_H
#define WRNCH_DEVICE_H

#include "IMU.h"
#include <array>

class Device {
private:
    IMU * imu;
    Device(const Device&);

public:
    Device();
    virtual ~Device() noexcept { delete imu; imu = nullptr;};

    IMU& getIMU();
    std::array<std::array<double,3>,3> getIMUdata();

};

#endif //WRNCH_DEVICE_H
