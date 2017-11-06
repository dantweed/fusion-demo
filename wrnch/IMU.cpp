//
// Created by Daniel.Tweed on 11/1/2017.
//

#include "IMU.h"

//Default constructor
IMU::IMU()  {
    //Initialize at rest and inline with x-y plane. Will be updated by sensor data
    acc ={0,0,0};
    gyro ={0,0,0};
    mag = {0,0,0};
}

std::array<double,3> IMU::getAcc() {
    std::array<double,3> rtnVal = acc;
    return rtnVal;
}
std::array<double,3> IMU::getGyro(){
    std::array<double,3> rtnVal = gyro;
    return rtnVal;
}

std::array<double,3> IMU::getMag(){
    std::array<double,3> rtnVal = mag;
    return rtnVal;
}

void IMU::setAcc(std::array<double,3> newAcc) {
    acc = newAcc;
}
void IMU::setGyro(std::array<double,3> newGyro) {
    gyro = newGyro;
}
void IMU::setMag(std::array<double,3> newMag){
    mag = newMag;
}