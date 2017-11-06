//
// Device.cpp
// An arbitrary device with on-board 9DOF IMU
//
// Created by Daniel.Tweed on 11/2/2017.
//

#include "Device.h"

//Default constructor
Device::Device() {
    imu = new IMU();
}

//Copy constructor defined due to pointer data member
Device::Device(const Device& copy) {
    imu = new IMU();
    imu = copy.imu;
}

/** Access and return available sensor data from IMU
 *
 * @return array of raw sensor readings
 *  Retrun value format: { {accX, accY, accZ},{gyroX, gyroY, gyroZ},{magX, magY, magZ} }
 */
std::array<std::array<double,3>,3> Device::getIMUdata(){
    std::array<std::array<double,3>,3> imuData= {imu->getAcc(),imu->getGyro(), imu->getMag()};
    return  imuData;
}

/** Helper for simulation purposes to access IMU to externally set sensor data
 *
 * @return reference to the on-board IMU object
 */
IMU& Device::getIMU() {
    return *(imu);
}
