#ifndef CODEPAMI_ROTATETOWARDPOSITIONTARGET_H
#define CODEPAMI_ROTATETOWARDPOSITIONTARGET_H
#include "Target.h"

class RotateTowardPositionTarget : public Target{
    Position pos;
    Ramp* ramp = nullptr;
    PRECISION_DATA_TYPE max_speed;
    PRECISION_DATA_TYPE acc;
    PRECISION_DATA_TYPE dec;
    bool done = false;
    PRECISION_DATA_TYPE target;
public:
    RotateTowardPositionTarget(Robot *robot, const Position& pos, PRECISION_DATA_TYPE acc=90, PRECISION_DATA_TYPE dec=90, PRECISION_DATA_TYPE max_speed=90);

    void init() override;

    bool is_done() override;

    void process() override;

    void on_done() override;

    void reinitRamp() override;

    Target* clone() const override {
        return new RotateTowardPositionTarget(*this);
    }

    ~RotateTowardPositionTarget() override;
};

#endif //CODEPAMI_ROTATETOWARDPOSITIONTARGET_H
