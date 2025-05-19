#ifndef CODEPAMI_BEZIERTARGET_H
#define CODEPAMI_BEZIERTARGET_H
#include "CurveTarget.h"

class BezierTarget : public CurveTarget {
protected:
    PRECISION_DATA_TYPE multiplier;
public:
    BezierTarget(Robot *robot, PRECISION_DATA_TYPE acc, PRECISION_DATA_TYPE max_speed, PRECISION_DATA_TYPE dec, Position end_pos, PRECISION_DATA_TYPE multiplier=100.0f, PRECISION_DATA_TYPE end_speed=0.0f);

    void init() override;

    void on_done() override;

    void reinitRamp() override;

};

#endif //CODEPAMI_ANGLETARGET_H
