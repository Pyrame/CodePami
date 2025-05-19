#include "./targets/BezierTarget.h"
#include "Robot.h"
#include "curves/BezierCurve.h"

BezierTarget::BezierTarget(Robot *robot, PRECISION_DATA_TYPE acc, PRECISION_DATA_TYPE max_speed, PRECISION_DATA_TYPE dec, Position end_pos, PRECISION_DATA_TYPE multiplier, PRECISION_DATA_TYPE end_speed): CurveTarget(robot, acc, max_speed, dec, (Position&&)end_pos, end_speed), multiplier(multiplier){

}

void BezierTarget::init(){
#ifdef DEBUG_TXT
    Serial.print("Init target toward : ");
    Serial.println(end_pos);
#endif
    this->curve = new BezierCurve(robot->getPosition(), end_pos, multiplier);
    this->distanceRamp = new Ramp(acc, max_speed, dec, this->curve->getLength(), this->robot->getRampSpeed(), end_speed);
    this->distanceRamp->start(robot->getTotalDistance());
    robot->setDoneAngle(false);
    robot->setDoneDistance(false);
    this->t = this->curve->getTForLength(NAN, 30.0f);
    this->target_pos = this->curve->getPosition(this->t);
}

void BezierTarget::on_done() {

}

void BezierTarget::reinitRamp(){
    delete distanceRamp;
    PRECISION_DATA_TYPE t = this->curve->findNearest(robot->getPosition());
    PRECISION_DATA_TYPE length = this->curve->getLength(t);
    Serial.print("T : ");
    Serial.println(t);
    Serial.print("Length : ");
    Serial.println(length);

    this->distanceRamp = new Ramp(acc, max_speed, dec, length, this->robot->getRampSpeed(), end_speed);
    this->distanceRamp->start(robot->getTotalDistance());
}


