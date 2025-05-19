#ifndef CODEPAMI_ARCCURVE_H
#define CODEPAMI_ARCCURVE_H
#include "curves/Curve.h"
class ArcCurve: public Curve {
public:
    ArcCurve(Position start_pos, Position end_pos);
};


#endif //CODEPAMI_ARCCURVE_H
