#include <Arduino.h>
#include "config.h"
#include "Position.h"
#include "Robot.h"
#include <Romi32U4.h>


//#define TRACK_MM 142.0f *3645/3600
#define TRACK_MM 140.2469135802469f
#define TICK_PER_MM 6.58f
#define Pin_PWM_Left 10 //PWM
#define Pin_Dir_Left 16
#define Pin_PWM_Right 9
#define Pin_Dir_Right 15
#include "Ramp.h"
#include "targets/BezierTarget.h"
#include "targets/PositionTarget.h"
#include "targets/RotateTowardPositionTarget.h"
#include "targets/AngleTarget.h"
#include <Wire.h>

Robot* robot;
Motor* left_motor = new Motor(Pin_Dir_Left, Pin_PWM_Left);
Motor* right_motor = new Motor(Pin_Dir_Right, Pin_PWM_Right);

//uint64_t last_time;
int freeMemory() {
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
uint64_t previous_micros;
void setup() {
    Serial.begin(9600);
    delay(5000);
    Position init_pos(-1000,0,0);
    Position pos(1000,1000,80);
    //last_time = micros();
    robot = new Robot(left_motor,right_motor, TICK_PER_MM, TRACK_MM, 1.0f);
    robot->setPidDistance(new PID(3, 0.1, 0.02));
    robot->setPidAngle(new PID(2, 0.02, 0.01));
    //robot->addTarget(new AngleTarget(robot, 360*40, 90,90,360,false));

    robot->addTarget(new PositionTarget(robot, init_pos, 300,300,400,300));
    robot->addTarget(new BezierTarget(robot, 300, 400, 300, pos, 3000));
    robot->addTarget(new BezierTarget(robot, 300,400,300,{2000, 1000, -80}, 1000));
    robot->addTarget(new BezierTarget(robot, 300,400,300,{3000, 1000, 80}, 1000));
    robot->addTarget(new AngleTarget(robot, 0));


    /*
    robot->addTarget(new RotateTowardPositionTarget(robot, {0,1000}));
    robot->addTarget(new PositionTarget(robot, {0,1000}, 300,400,300));
    robot->addTarget(new RotateTowardPositionTarget(robot, {0,0}));
    robot->addTarget(new PositionTarget(robot, {0,0}, 300, 400, 300));
     */
    previous_micros = micros();
    //robot->addTarget(new AngleTarget(robot, 360));
    //robot->addTarget(new AngleTarget(robot, -170));
    //robot->addTarget(new RotateTowardPositionTarget(robot, pos));
    //robot->addTarget(new PositionTarget(robot, pos, 300, 400, 300, 300));
    //robot->addTarget(new PositionTarget(robot, pos, 300, 600, 150));
// write your initialization code here
}

void loop() {
    while(micros() - previous_micros < 5000);
    previous_micros = micros();
    delayMicroseconds(5000);
    robot->computePosition(Romi32U4Encoders::getCountsAndResetLeft(), Romi32U4Encoders::getCountsAndResetRight());
    robot->computeTarget();
    robot->control();
    //uint64_t next = micros();
    //Serial.println(((double)(next-last_time))/1000.0f);
    //last_time = next;
#ifdef DEBUG_TXT
    Serial.println(*robot);
#endif
    //Serial.println(freeMemory());
// write your code here
}
