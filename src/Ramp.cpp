#include "Ramp.h"

#define SIGN(x) (x > 0 ? 1 : -1)

Ramp::Ramp(PRECISION_DATA_TYPE acc, PRECISION_DATA_TYPE max_speed, PRECISION_DATA_TYPE dec, PRECISION_DATA_TYPE distance, PRECISION_DATA_TYPE initial_speed, PRECISION_DATA_TYPE end_speed) {
    this->acc = acc;
    this->dec = dec;

    this->sign = SIGN(distance);

    this->sign_corrected.initial_speed = this->sign * initial_speed;
    this->sign_corrected.current_speed = this->sign_corrected.initial_speed;
    this->sign_corrected.end_speed = this->sign * end_speed;

    this->acc_time = (max_speed - this->sign_corrected.initial_speed)/acc;
    this->dec_time = (max_speed - this->sign_corrected.end_speed)/dec;
    this->sign_corrected.acc_distance = this->sign_corrected.initial_speed * abs(this->acc_time) + SIGN(this->acc_time) * acc * pow(this->acc_time, 2)/2.0f;
    this->sign_corrected.dec_distance = max_speed * abs(this->dec_time) - SIGN(this->dec_time) * dec * pow(this->dec_time, 2)/2.0;

    if(this->sign_corrected.acc_distance + this->sign_corrected.dec_distance <= abs(distance)){
        this->ste_time = (abs(distance) - this->sign_corrected.acc_distance - this->sign_corrected.dec_distance)/max_speed;
        this->ste_speed = this->sign * max_speed;
    }else{

        this->acc_time = -this->sign_corrected.initial_speed/acc + sqrt((acc + dec) * (2*acc*dec* abs(distance) + acc * pow(end_speed, 2) + dec * pow(initial_speed, 2)))/(acc * (acc+dec));
        this->dec_time = (acc * this->acc_time + this->sign_corrected.initial_speed - this->sign_corrected.end_speed)/dec;
        this->ste_time = 0;
        this->ste_speed = sqrt((2*acc * abs(distance) * dec + acc * pow(this->sign_corrected.end_speed, 2) + dec * pow(this->sign_corrected.initial_speed, 2))/(acc+dec));
    }
    this->sign_corrected.acc_distance = this->sign_corrected.initial_speed * abs(this->acc_time) + SIGN(this->acc_time) * acc * pow(this->acc_time, 2)/2.0f;
    this->sign_corrected.dec_distance = ste_speed * abs(this->dec_time) - SIGN(this->dec_time) * dec * pow(this->dec_time, 2)/2.0;

    this->sign_corrected.point_distance = abs(distance);
    this->init_ste_time = this->ste_time;
#ifdef DEBUG_TXT
    Serial.print("NEW RAMP acc_time : ");
    Serial.print(acc_time);
    Serial.print(", dec_time : ");
    Serial.print(dec_time);
    Serial.print(", ste_time : ");
    Serial.print(ste_time);
    Serial.print(", ste_speed : ");
    Serial.print(ste_speed);
    Serial.print(", acc_distance : ");
    Serial.print(this->sign_corrected.acc_distance);
    Serial.print(", dec_distance : ");
    Serial.print(this->sign_corrected.dec_distance);
    Serial.print(", acc : ");
    Serial.print(acc);
    Serial.print(", dec : ");
    Serial.print(dec);
    Serial.print(", init_speed : ");
    Serial.print(this->sign_corrected.initial_speed);
    Serial.print(", dist : ");
    Serial.println(distance);
#endif


}

void Ramp::start(PRECISION_DATA_TYPE start_wheel) {
    this->init_time = micros();
    this->sign_corrected.wheel_distance = sign * start_wheel;
}

const RampReturnData Ramp::compute(PRECISION_DATA_TYPE distance_to_point, PRECISION_DATA_TYPE wheel_distance) {
    if(this->init_time == 0){
        return {false, 0, false, false};
    }
    RampReturnData returndata;
    PRECISION_DATA_TYPE time = ((PRECISION_DATA_TYPE)(micros() - this->init_time))/1e6;
    if(increment)
        update_ste_time(time, distance_to_point, wheel_distance);
    Ramp::Private data = compute_at_time(time);
    if(!isnan(previous_computation.speed)){
        returndata = {data.done,data.distance - previous_computation.distance, data.speed, data.done && data.speed == 0.0, true};
    }else{
        returndata =  {data.done,data.distance, data.speed, data.done && data.speed == 0.0, true};
    }
    previous_computation = data;
    return returndata;
}

const Ramp::Private Ramp::compute_at_time(PRECISION_DATA_TYPE time) const {
    PRECISION_DATA_TYPE current_speed;
    PRECISION_DATA_TYPE distance;
    bool done = false;
    if(time < acc_time){
        current_speed = sign_corrected.initial_speed + acc * time;
        distance = sign_corrected.initial_speed*time + acc * pow(time, 2)/2.0f;
    }else if(time < acc_time + ste_time){
        current_speed = ste_speed;
        distance = sign_corrected.initial_speed * acc_time + acc * pow(acc_time, 2)/2.0f + (time - acc_time) * ste_speed;
    }else if(time < acc_time + ste_time + dec_time){
        current_speed = ste_speed - dec*(time - acc_time - ste_time);
        distance = sign_corrected.initial_speed * acc_time + acc * pow(acc_time, 2)/2.0f + ste_time * ste_speed + ste_speed * (time - acc_time - ste_time) - dec*pow(time - acc_time - ste_time, 2)/2.0f;
    }else{
        current_speed = this->sign_corrected.end_speed;
        distance = sign_corrected.initial_speed * acc_time + acc * pow(acc_time, 2)/2.0f + ste_time * ste_speed + ste_speed * dec_time - dec*pow(dec_time, 2)/2.0f;
        done = true;
    }
    return {done, current_speed * this->sign, distance * this->sign};
}

void Ramp::update_ste_time(PRECISION_DATA_TYPE time, PRECISION_DATA_TYPE distance, PRECISION_DATA_TYPE wheel_distance) {

    if(time > acc_time && time < acc_time + ste_time)
        ste_time = init_ste_time + ((sign * wheel_distance - sign_corrected.wheel_distance) + sign * distance - sign_corrected.point_distance)/ste_speed;

}

const RampReturnData Ramp::compute() {
    if(this->init_time == 0){
        return {false, 0, false, false};
    }
    RampReturnData returndata;
    PRECISION_DATA_TYPE time = ((PRECISION_DATA_TYPE)(micros() - this->init_time))/1e6;
    Ramp::Private data = compute_at_time(time);
    if(!isnan(previous_computation.speed)){
        returndata = {data.done,data.distance - previous_computation.distance, data.speed, data.done && data.speed == 0.0, true};
    }else{
        returndata =  {data.done,data.distance, data.speed, data.done && data.speed == 0.0, true};
    }
    previous_computation = data;
    return returndata;
}
