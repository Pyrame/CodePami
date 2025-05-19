#include "./targets/PositionTarget.h"
#include "Robot.h"

PositionTarget::PositionTarget(Robot *robot, const Position &pos, PRECISION_DATA_TYPE acc, PRECISION_DATA_TYPE dec, PRECISION_DATA_TYPE max_speed, PRECISION_DATA_TYPE end_speed) : Target(robot), pos(pos), max_speed(max_speed), acc(acc), dec(dec), end_speed(end_speed) {}

void PositionTarget::init() {
    ramp = new Ramp(acc, max_speed, dec, (pos-robot->getPosition()).getDistance(), robot->getRampSpeed(), end_speed);
    ramp->start(robot->getTotalDistance());
    robot->setDoneAngle(false);
    robot->setDoneDistance(false);
    robot->setTotalAngle(robot->getPosition().getAngle());
}

bool PositionTarget::is_done() {
    return done;
}

void PositionTarget::process() {
    PRECISION_DATA_TYPE distance = (pos-robot->getPosition()).getDistance();
    RampReturnData data = ramp->compute(distance, robot->getTotalDistance());
    robot->setRampSpeed(data.speed);
    robot->setTargetDistance(robot->getTargetDistance() + data.distance_increment);
    robot->setTargetAngle(correctAngle((pos-robot->getPosition()).getVectorAngle()));
    done = data.end;
    robot->setDoneDistance(data.stop);
    if(distance < 5) {
        if(end_speed == 0.0f)
            robot->setTargetDistance(robot->getTotalDistance());
        robot->setDoneAngle(true);
    }else{
        robot->setDoneAngle(false);
    }
}

PositionTarget::~PositionTarget() {
    delete ramp;
}

void PositionTarget::on_done() {
    if(end_speed == 0.0f)
        robot->setTargetDistance(robot->getTotalDistance());
    robot->setTargetAngle(robot->getTotalAngle());
}

void PositionTarget::reinitRamp(){
    delete ramp;
    ramp = new Ramp(acc, max_speed, dec, (pos-robot->getPosition()).getDistance(), robot->getRampSpeed(), end_speed);
    ramp->start(robot->getTotalDistance());
}

