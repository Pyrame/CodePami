//
// Created by fogoz on 13/12/2024.
//

#ifndef CODEPAMI_CURVE_H
#define CODEPAMI_CURVE_H
#include "Position.h"

class Curve {
protected:
    double min_value;
    double max_value;

public:
    Curve(PRECISION_DATA_TYPE min_value, PRECISION_DATA_TYPE max_value);

    virtual Position getPosition(PRECISION_DATA_TYPE time) const = 0;
    virtual Position getDerivative(PRECISION_DATA_TYPE time) const = 0;

    PRECISION_DATA_TYPE getLength(PRECISION_DATA_TYPE t0 = 0, PRECISION_DATA_TYPE h=0.01f) const;

    PRECISION_DATA_TYPE getTForLength(PRECISION_DATA_TYPE ti=NAN, PRECISION_DATA_TYPE length=100.0f, PRECISION_DATA_TYPE h=0.01f) const;

    PRECISION_DATA_TYPE findNearest(Position pos, PRECISION_DATA_TYPE hmax=0.01f) const;

    virtual ~Curve();
};


#endif //CODEPAMI_CURVE_H
