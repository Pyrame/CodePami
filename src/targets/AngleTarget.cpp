#include "./targets/AngleTarget.h"
#include "Robot.h"


bool AngleTarget::is_done() {
    return done;
}

void AngleTarget::process() {
    RampReturnData data = ramp->compute();
    robot->setRampSpeedAngle(data.speed);
    robot->setTargetAngle(robot->getTargetAngle() + data.distance_increment);
    done = data.end;
}

void AngleTarget::init() {
    if(!optimized){
        ramp = new Ramp(acc, max_speed, dec, target-robot->getPosition().getAngle());
    }else{
        ramp = new Ramp(acc, max_speed, dec, correctAngle(target-robot->getPosition().getAngle()));
    }
    ramp->start(robot->getTotalAngle());
    robot->setDoneAngle(false);
    robot->setDoneDistance(true);
}

AngleTarget::~AngleTarget() {
    delete ramp;
}

AngleTarget::AngleTarget(Robot *robot, PRECISION_DATA_TYPE target, PRECISION_DATA_TYPE acc, PRECISION_DATA_TYPE dec, PRECISION_DATA_TYPE max_speed, bool optimized) : Target(robot) {
    this->target = target;
    this->acc = acc;
    this->dec = dec;
    this->max_speed = max_speed;
    this->optimized = optimized;
}

void AngleTarget::on_done() {
    robot->setTargetDistance(robot->getTotalDistance());
}

void AngleTarget::reinitRamp(){
    delete ramp;
    if(!optimized){
        ramp = new Ramp(acc, max_speed, dec, target-robot->getPosition().getAngle());
    }else{
        ramp = new Ramp(acc, max_speed, dec, correctAngle(target-robot->getPosition().getAngle()));
    }
    ramp->start(robot->getTotalAngle());
}
