//
// MDKalman.h
// Wrapper for several independent Kalman filters
//
// Created by Daniel.Tweed on 11/2/2017.
//

#ifndef WRNCH_MDKALMAN_H
#define WRNCH_MDKALMAN_H

#include <vector>
#include <array>
#include "kalman.h"
#include <cstdint>

template <uint16_t N> class MDKalman {
public:
    MDKalman (){
        for (int i = 0; i < N; ++i)
            filters.push_back(new Kalman());
    }

    explicit MDKalman(kalman_state init_state ) {
        for (int i = 0; i < N; ++i)
            filters.push_back(new Kalman(init_state));
    }

    explicit MDKalman(std::array<kalman_state,N> init_states ) {
        for (int i = 0; i < N; ++i)
            filters.push_back(new Kalman(init_states[i]));
    }

    MDKalman(const MDKalman& copy){
        for (auto filter_p: copy.filters)
            filters.push_back(new Kalman(filter_p->state));
    }

    ~MDKalman(){
        for (auto filter_p : filters)
            delete filter_p;
    }

    void init(std::array<double,N> q, std::array<double,N> r, std::array<double,N> p, std::array<double,N> x_init ){
        for (int i = 0; i < N; ++i)
            filters[i]->kalman_init(q[i],r[i],p[i],x_init[i]);
    }

    void init(double q, double r, double p, std::array<double,N> x_init ){
        for (int i = 0; i < N; ++i)
            filters[i]->kalman_init(q,r,p,x_init[i]);
    }

    void init(double q, double r, double p, double x_init ){
        for (auto filter_p : filters)
            filter_p->kalman_init(q,r,p,x_init);
    }

    void update(std::array<double,N> measurements){
        for (uint16_t i = 0; i < N; ++i)
            filters.at(i)->update(measurements[i]);
    }

    std::array<double,N> filtered(){
        std::array<double,N> rtnVal;
        for (uint16_t i = 0; i < N; ++i)
            rtnVal[i] = filters.at(i)->filtered();
        return rtnVal;
    }

private:
    std::vector<Kalman*> filters;
};
#endif //WRNCH_MDKALMAN_H
