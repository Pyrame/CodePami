#ifndef CODEPAMI_POSITIONTARGET_H
#define CODEPAMI_POSITIONTARGET_H
#include "Target.h"

class PositionTarget : public Target{
    Position pos;
    Ramp* ramp = nullptr;
    PRECISION_DATA_TYPE max_speed;
    PRECISION_DATA_TYPE acc;
    PRECISION_DATA_TYPE dec;
    bool done = false;
    PRECISION_DATA_TYPE end_speed;
    bool backward = false;
public:
    PositionTarget(Robot *robot, const Position &pos, PRECISION_DATA_TYPE acc=100, PRECISION_DATA_TYPE dec=100, PRECISION_DATA_TYPE max_speed=200, PRECISION_DATA_TYPE end_speed=0.0);

    void init() override;

    bool is_done() override;

    void process() override;

    void reinitRamp() override;

    Target* clone() const override {
        return new PositionTarget(robot, pos, acc, dec, max_speed, end_speed);
    }

    Target* generateRotateToward() override;

    ~PositionTarget() override;

    void on_done() override;
};
#endif //CODEPAMI_POSITIONTARGET_H
