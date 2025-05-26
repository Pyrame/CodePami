#include "./targets/RotateTowardPositionTarget.h"
#include "Robot.h"


void RotateTowardPositionTarget::init() {
    if(ramp != nullptr){
        delete ramp;
    }
    target = (pos-robot->getPosition()).getVectorAngle();
#ifdef DEBUG_TXT
    Serial.println(target- robot->getTargetAngle());
    Serial.println(correctAngle(target- robot->getTargetAngle()));
#endif
    ramp = new Ramp(acc, max_speed, dec, correctAngle(target- robot->getTargetAngle()));
    ramp->start(robot->getTotalAngle());
    robot->setDoneAngle(false);
    robot->setDoneDistance(true);
}

bool RotateTowardPositionTarget::is_done() {
    return done;
}

void RotateTowardPositionTarget::process() {
    RampReturnData data = ramp->compute();
    robot->setRampSpeedAngle(data.speed);
    robot->setTargetAngle(robot->getTargetAngle() + data.distance_increment);
    done = data.end;
}

RotateTowardPositionTarget::~RotateTowardPositionTarget() {
    delete ramp;
}

RotateTowardPositionTarget::RotateTowardPositionTarget(Robot *robot, const Position &pos, PRECISION_DATA_TYPE acc, PRECISION_DATA_TYPE dec,
                                                       PRECISION_DATA_TYPE max_speed): Target(robot), pos(pos), max_speed(max_speed), acc(acc), dec(dec) {

}

void RotateTowardPositionTarget::on_done() {
    robot->setTargetDistance(robot->getTotalDistance());
#ifdef DEBUG_TXT
    Serial.println("Called rotate toward");
#endif
}

void RotateTowardPositionTarget::reinitRamp() {
    delete ramp;
    target = (pos-robot->getPosition()).getVectorAngle();
    ramp = new Ramp(acc, max_speed, dec, correctAngle(target- robot->getTargetAngle()));
    ramp->start(robot->getTotalAngle());

}
