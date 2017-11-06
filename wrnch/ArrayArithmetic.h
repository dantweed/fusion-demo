//
// Helper templates for array arithmetic
//
// Created by Daniel.Tweed on 11/5/2017.
//

#ifndef WRNCH_ARRAYARITHMETIC_H
#define WRNCH_ARRAYARITHMETIC_H

namespace wrnch {
    template<class T>
    T operator+(const T &a1, const T &a2) {
        T a;
        for (typename T::size_type i = 0; i < a1.size(); i++)
            a[i] = a1[i] + a2[i];
        return a;
    }

    template<class T>
    T operator-(const T &a1, const T &a2) {
        T a;
        for (typename T::size_type i = 0; i < a1.size(); i++)
            a[i] = a1[i] - a2[i];
        return a;
    }

    template<class T>
    T operator*(const T &a1, double c) {
        T a;
        for (typename T::size_type i = 0; i < a1.size(); i++)
            a[i] = a1[i] * c;
        return a;
    }

    template<class T>
    T operator*(double c, const T &a1) {
        T a;
        for (typename T::size_type i = 0; i < a1.size(); i++)
            a[i] = a1[i] * c;
        return a;
    }

}
#endif //WRNCH_ARRAYARITHMETIC_H
