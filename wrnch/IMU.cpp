//
// Created by Daniel.Tweed on 11/1/2017.
//

#include "IMU.h"

//Default constructor
IMU::IMU()  {    
    acc ={0,0,0}; //Initialize at rest and inline with x-y plane
    gyro ={0,0,0};// Will be updated by sensor data on read
    mag = {0,0,0};
}

/** Access and return available sensor data from accelerometer
 *
 * @return array of raw sensor readings
 *  Retrun value format: { {accX, accY, accZ} }
 */
std::array<double,3> IMU::getAcc() {
    std::array<double,3> rtnVal = acc;
    return rtnVal;
}

/** Access and return available sensor data from accelerometer
 *
 * @return array of raw sensor readings
 *  Retrun value format: { {gyroX, gyroY, gyroZ} }
 */
std::array<double,3> IMU::getGyro(){
    std::array<double,3> rtnVal = gyro;
    return rtnVal;
}

/** Access and return available sensor data from magnetometer
 *
 * @return array of raw sensor readings
 *  Retrun value format: { {magX, magY, magZ} }
 */
std::array<double,3> IMU::getMag(){
    std::array<double,3> rtnVal = mag;
    return rtnVal;
}

/** Set acceleromter
 *
 * @param newAcc
 */
void IMU::setAcc(std::array<double,3> newAcc) {
    acc = newAcc;
}
void IMU::setGyro(std::array<double,3> newGyro) {
    gyro = newGyro;
}
void IMU::setMag(std::array<double,3> newMag){
    mag = newMag;
}
