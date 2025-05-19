#include "Arduino.h"
#include "Target.h"
#include "PID.h"

#ifndef ROBOT_H
#define ROBOT_H

PRECISION_DATA_TYPE correctAngle(PRECISION_DATA_TYPE angle);
class Motor{
    uint8_t dir_pin;
    uint8_t pwm_pin;
    int16_t current_pwm = 0;
    bool inverse;
public:
    Motor(uint8_t dir_pin, uint8_t pwm_pin, bool inverse=false);

    void setPWM(int16_t pwm);

};


class Robot : public Printable{
    Position pos;
    PRECISION_DATA_TYPE pulse_per_mm;
    PRECISION_DATA_TYPE track_mm;
    PRECISION_DATA_TYPE corr_right_wheel;
    Motor* left_motor = nullptr;
    Motor* right_motor = nullptr;
    uint16_t target_count = 0;
    Target** targets = nullptr;
    uint16_t max_targets = 10;
    uint16_t target_index = 0;
    PRECISION_DATA_TYPE total_distance;
    PRECISION_DATA_TYPE target_distance;
    PRECISION_DATA_TYPE total_angle;
    PRECISION_DATA_TYPE target_angle;
    PID* pid_distance = nullptr;
    PID* pid_angle = nullptr;
    bool done_distance = false;
    bool done_angle = false;

    PRECISION_DATA_TYPE ramp_speed = 0.0f;
    PRECISION_DATA_TYPE ramp_speed_angle = 0.0f;

    friend Target;

    public:
    Robot(Motor* left_motor, Motor* right_motor, PRECISION_DATA_TYPE pulse_per_mm, PRECISION_DATA_TYPE track_mm, PRECISION_DATA_TYPE corr_right_wheel, PRECISION_DATA_TYPE x=0.0f, PRECISION_DATA_TYPE y=0.0f, PRECISION_DATA_TYPE a=0.0f);

    ~Robot();
    
    void clearTargets();

    void computePosition(int16_t delta_left_tick, int16_t delta_right_tick);

    [[nodiscard]] Position getPosition() const;

    void control();

    void setPidDistance(PID *pidDistance);

    void setPidAngle(PID *pidAngle);

    void computeTarget();

    bool addTarget(Target* target);

    void resetTarget();

    size_t printTo(Print &p) const override;

    PRECISION_DATA_TYPE getTargetDistance() const;

    PRECISION_DATA_TYPE getTargetAngle() const;

    PRECISION_DATA_TYPE getTotalDistance() const;

    PRECISION_DATA_TYPE getTotalAngle() const;

    bool isDoneDistance() const;

    bool isDoneAngle() const;

    void setTargetDistance(PRECISION_DATA_TYPE targetDistance);

    void setTargetAngle(PRECISION_DATA_TYPE targetAngle);

    void setDoneDistance(bool doneDistance);

    void setDoneAngle(bool doneAngle);

    PRECISION_DATA_TYPE getRampSpeed() const;

    void setRampSpeed(PRECISION_DATA_TYPE rampSpeed);

    PRECISION_DATA_TYPE getRampSpeedAngle() const;

    void setRampSpeedAngle(PRECISION_DATA_TYPE rampSpeedAngle);

    PRECISION_DATA_TYPE getRelativeAngle(PRECISION_DATA_TYPE a) const;

    PRECISION_DATA_TYPE getAbsoluteAngle(PRECISION_DATA_TYPE a) const;

    void setTotalDistance(PRECISION_DATA_TYPE totalDistance);

    void setTotalAngle(PRECISION_DATA_TYPE totalAngle);

    void stop();

    void resetcontrol();
};
#endif