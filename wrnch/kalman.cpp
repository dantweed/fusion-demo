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

/** Set initial values for filter
 *
 * @param q process noise covariance
 * @param r measurement noise covariance
 * @param p estimation error covariance
 * @param x_init initial value for parameter being filtered
 */
void Kalman::kalman_init (double q, double r, double p, double x_init){
    state.q = q;
    state.r = r;
    state.p = p;
    state.x = x_init;
    state.k = 0;
}

/** Update with new measurement
 *
 * @param measurement of parameter being filtered
 */
void Kalman::update(double measurement)
{
    //prediction update
    state.p = state.p + state.q;

    //measurement update
    state.k = state.p / (state.p + state.r);
    state.x = state.x + state.k * (measurement - state.x);
    state.p = (1 - state.k) * state.p;    
}

/** Access and return current estimate from filter
 *
 * @return current estimate
 */
double Kalman::filtered() {
    return state.x;
}
