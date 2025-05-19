#ifndef COURBEBEZIER_H
#define COURBEBEZIER_H

#include "Position.h"
#include "Curve.h"

class BezierCurve: public Curve {
    Position pos1;
    Position pos2;
    Position pos3;
    Position pos4;
    public:
    explicit BezierCurve(Position start, Position end, PRECISION_DATA_TYPE multiplier=100.0f);

    Position getPosition(PRECISION_DATA_TYPE time) const;

    Position getDerivative(PRECISION_DATA_TYPE time) const;

};



#endif //COURBEBEZIER_H
