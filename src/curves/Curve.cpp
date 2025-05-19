//
// Created by fogoz on 13/12/2024.
//

#include "curves/Curve.h"
#include "Integrator.h"

Curve::Curve(PRECISION_DATA_TYPE min_value, PRECISION_DATA_TYPE max_value) : min_value(min_value), max_value(max_value){}

PRECISION_DATA_TYPE Curve::getLength(PRECISION_DATA_TYPE t0, PRECISION_DATA_TYPE h) const {
    PRECISION_DATA_TYPE y0 = 0;
    runge_kutta_4(t0, &y0, 1, 1.0, h, [this](PRECISION_DATA_TYPE t, const PRECISION_DATA_TYPE* y, PRECISION_DATA_TYPE* dydt, int n) {
        dydt[0] = 0 + this->getDerivative(t).getDistance();
    });
    return y0;
}

PRECISION_DATA_TYPE Curve::getTForLength(PRECISION_DATA_TYPE ti, PRECISION_DATA_TYPE length, PRECISION_DATA_TYPE h) const {
    if(isnan(ti))
        ti = min_value;
    PRECISION_DATA_TYPE y0 = 0;
    auto lambda = [this](PRECISION_DATA_TYPE t, const PRECISION_DATA_TYPE* y, PRECISION_DATA_TYPE* dydt, int n) {
        dydt[0] = 0 + this->getDerivative(t).getDistance();
    };
    return runge_kutta_4_maximized(ti, &y0, 1, 1.0, h, lambda, &length);
}

PRECISION_DATA_TYPE Curve::findNearest(Position pos, PRECISION_DATA_TYPE hmax) const{
    PRECISION_DATA_TYPE dist = (getPosition(1.0f) - pos).getDistance();
    PRECISION_DATA_TYPE index = 1.0f;
    for(PRECISION_DATA_TYPE data_type = 0; data_type < 1.0f; data_type+=hmax){
        PRECISION_DATA_TYPE c_dist = (getPosition(data_type) - pos).getDistance();
        if(c_dist < dist){
            index = data_type;
        }
    }
    return index;
}


Curve::~Curve() {

}
