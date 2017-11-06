//
// kalmna.cpp
// Linear 1-D Kalman Filter
//
// Created by Daniel.Tweed on 11/2/2017.
//

#include "kalman.h"

//Default constructor
Kalman::Kalman() {}

//Construct from defined initial state
Kalman::Kalman(kalman_state initState) {
    state = initState;}

//Construct from initial values
Kalman::Kalman(double q, double r, double p, double x_init){
    kalman_init (q, r, p, x_init);
}

//TODO: Comments
void Kalman::kalman_init (double q, double r, double p, double x_init){
    state.q = q;  //process noise covariance
    state.r = r;  //measurement noise covariance
    state.p = p;  //estimation error covariance
    state.x = x_init;  //value
    state.k = 0;  //kalman gain
}


void Kalman::update(double measurement)
{
    //prediction update
    state.p = state.p + state.q;

    //measurement update
    state.k = state.p / (state.p + state.r);
    state.x = state.x + state.k * (measurement - state.x);
    state.p = (1 - state.k) * state.p;
}

double Kalman::filtered() {
    return state.x;
}