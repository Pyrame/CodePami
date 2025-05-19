#include "PID.h"

PID::PID(PRECISION_DATA_TYPE kp, PRECISION_DATA_TYPE ki, PRECISION_DATA_TYPE kd, PRECISION_DATA_TYPE (*anti_windup)(PRECISION_DATA_TYPE)){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->anti_windup = anti_windup;
    this->old_error = 0.0f;
    this->iTerm = 0.0f;
    this->previous_time = (PRECISION_DATA_TYPE)micros();
}

PRECISION_DATA_TYPE PID::evaluate(PRECISION_DATA_TYPE error){
    PRECISION_DATA_TYPE result = 0;
    PRECISION_DATA_TYPE dt = ((PRECISION_DATA_TYPE)micros() - this->previous_time)/(1000000.0f);
    this->previous_time = (PRECISION_DATA_TYPE)micros();
    iTerm += this->ki * error * dt;
    iTerm = anti_windup(iTerm);

    result += this->kp * error + kd * (error - this->old_error)/dt + iTerm;
    this->old_error = error;
    return result;
}

void PID::resetTerms(){
    iTerm = 0;
    this->old_error = 0;
}

// PRECISION_DATA_TYPE PID::resetPID(PRECISION_DATA_TYPE error){
//     PRECISION_DATA_TYPE result = 0;
//     PRECISION_DATA_TYPE dt = ((PRECISION_DATA_TYPE)micros() - this->previous_time)/(1000000.0f);
//     this->previous_time = (PRECISION_DATA_TYPE)micros();
//     iTerm += this->ki * error * dt;
//     iTerm = anti_windup(iTerm);

//     result += this->kp * error + kd * (error - this->old_error)/dt + iTerm;
//     this->old_error = error;
//     return result;
// }