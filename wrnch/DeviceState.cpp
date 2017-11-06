//
// DeviceState.cpp
// Calculate and present the properties of a device in 3-D space, relative to its own reference frame
// from IMU sensor data.
//   velocity, position, orientation.
// Created by Daniel.Tweed on 11/2/2017.
//
#include "DeviceState.h"

using namespace wrnch;

DeviceState::DeviceState(Device & dev_, float dT) : dev(dev_), deltaT(dT){

    //TODO: Validate initial values for k_state
    std::array<std::array<double,3> ,3>  initData = dev.getIMUdata();
    acc.init(0.1,1,0,initData[0]);

    //Calibrate to intial readings (i.e. assume gyro/mag readings to  be correct/calibrated)
    gyro.init(0.1,1,0,initData[1]);//double q, double r, double p, double x_init
    mag.init(0.1,1,0,initData[2]);

    estPos = {0,0,0};
    estVel = {0,0,0};
    //estOrient initialized to initialized to (1,0,0,0) by etk::Quaternion()
}

const double DeviceState::G = 9.8; //Gravitational acceleration m/s^2

/** Access and return current estimated velocity
 *
 * @return velocity in 3d array (v_x,v_y,v_z)
 */
std::array<double,3> DeviceState::getEstVel() {
    return estVel;
};

/** Access and return current estimated
 *
 * @return position in 3d array (p_x,p_y,p_z)
 */
std::array<double,3> DeviceState::getEstPos(){
    return estPos;
};

/** Access and return current stimated orientation in Euler angles
 *
 * @return array of euler angles in radians (x,y,z)
 */
std::array<double,3> DeviceState::getEstOrientAngles() {
    std::array<double,3> ret = {0,0,0};
    etk::Vector<3> euler = estOrient.toEuler();

    //Convert from etk::Vector to std::array
    ret[0]=euler.get_x();
    ret[1]=euler.get_y();
    ret[2]=euler.get_z();

    return ret;
}

/** Access and return current stimated orientation as quaternion
 *
 * @return orientation quaternion
 */
etk::Quaternion DeviceState::getEstOrientQ(){
        return estOrient;
}

/** Updates device state from IMU data
 *
 */
void DeviceState::updateFromIMU(std::array<std::array<double,3>,3> imuData){
    //std::array<std::array<double,3>,3> imuData = dev.getIMUdata();

    std::array<double,3> accRead = imuData[0];
    std::array<double,3> gyroRead =  imuData[1];
    std::array<double,3> magRead = imuData[2];

    //Update Kalman filter predictions

    acc.update(accRead);
    gyro.update(gyroRead);
    mag.update(magRead);

    //Update estimates
    updateOrient();
    updateVel();
    updatePos();
}

/** Updates device velocity in its own reference frame based on current velocity
 *  and most recent filtered acceleration data
 */
void DeviceState::updateVel() {
    //Estimated influence of gravity on accelerometer reading
    // i.e. calculate free acceleration
    std::array<double,3> g= {0.0,0.0,0.0};

    //Calculated from quaternion representing current orientation of the device
    g[0] = 2*(estOrient.x()*estOrient.z() - estOrient.w()*estOrient.y());
    g[1] = 2*(estOrient.w()*estOrient.x()+ estOrient.y()*estOrient.z());
    g[2] = estOrient.w()*estOrient.w() - estOrient.x()*estOrient.x()
            - estOrient.y()*estOrient.y() + estOrient.z()*estOrient.z();

    std::array<double,3> accData = acc.filtered();

    estVel  = estVel + (accData - g)*G*deltaT;
}

/** Updates device position in its own reference frame based on current position
 *  and velocity calculated
 */
void DeviceState::updatePos(){
    estPos = estPos + estVel*deltaT;
}

/** Updates device oritentation in its own reference frame from
 *  accelerometer, gryo and magnetometer data
 */
void DeviceState::updateOrient() {
    std::array<double,3> accData = acc.filtered();
    std::array<double,3> gyroData = gyro.filtered();
    std::array<double,3> magData = mag.filtered();

    etk::Vector<3> down(accData[0],accData[1],accData[2]);
    etk::Vector<3> east = down.cross(etk::Vector<3>(magData[0],magData[1],magData[2]));
    etk::Vector<3> north = east.cross(down);

    down.normalize();
    east.normalize();
    north.normalize();

    //Generate rotation matrix from accelerometer and gyro readings
    etk::Matrix<3,3> rotationMatrix;
    rotationMatrix.vector_to_col(north, 0);
    rotationMatrix.vector_to_col(east, 1);
    rotationMatrix.vector_to_col(down, 2);

    etk::Quaternion q_accel;
    q_accel.fromMatrix(rotationMatrix);

    //Estimate change in orientation from gyros angular velocity
    etk::Vector<3> w(gyroData[0],gyroData[1],gyroData[2]);
    etk::Quaternion qw;
    qw.fromAngularVelocity(w,deltaT);
    qw.normalize();

    //Update estimate
    estOrient = estOrient*qw;

    //Use Spherical Linear Interpolation to move estimate closer to q_accel estimate to combat gryo drift
    estOrient = estOrient.slerp(q_accel, 0.05); //TODO:ORIENTATION_FILTER_GAIN= 0.05 tbd if this is good or not
}


