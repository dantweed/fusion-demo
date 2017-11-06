//
// kalmna.h
// Linear 1-D Kalman Filter
//
// Created by Daniel.Tweed on 11/2/2017.
//

#ifndef WRNCH_KALMAN_H
#define WRNCH_KALMAN_H

#include <array>

typedef struct {
    double q; //process noise covariance
    double r; //measurement noise covariance
    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain
} kalman_state;

class Kalman {
private:
    kalman_state state;

public:
    Kalman();
    Kalman(kalman_state);
    Kalman(double q, double r, double p, double x_init);
    void kalman_init (double,double,double,double);
    void update(double);
    double filtered();
};
#endif //WRNCH_KALMAN_H
