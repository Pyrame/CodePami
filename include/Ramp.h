#include "Arduino.h"

#ifndef CODEPAMI_RAMP_H
#define CODEPAMI_RAMP_H
#include "config.h"


struct RampReturnData{
    bool end;
    PRECISION_DATA_TYPE distance_increment;
    PRECISION_DATA_TYPE speed;
    bool stop;
    bool started;
};

class Ramp {
    PRECISION_DATA_TYPE acc;
    PRECISION_DATA_TYPE dec;
    PRECISION_DATA_TYPE sign;
    PRECISION_DATA_TYPE ste_speed;

    bool increment;

    struct Private{
        bool done;
        PRECISION_DATA_TYPE speed;
        PRECISION_DATA_TYPE distance;
    };

    PRECISION_DATA_TYPE dec_time;
    PRECISION_DATA_TYPE acc_time;
    PRECISION_DATA_TYPE ste_time;

    PRECISION_DATA_TYPE init_ste_time;

    uint64_t init_time = 0;

    Ramp::Private previous_computation = {false, NAN, NAN};

    struct{
        PRECISION_DATA_TYPE initial_speed;
        PRECISION_DATA_TYPE current_speed;
        PRECISION_DATA_TYPE end_speed;
        PRECISION_DATA_TYPE acc_distance;
        PRECISION_DATA_TYPE dec_distance;

        PRECISION_DATA_TYPE point_distance = 0;
        PRECISION_DATA_TYPE wheel_distance = 0;
    }sign_corrected;
    const Private compute_at_time(PRECISION_DATA_TYPE time) const;
    void update_ste_time(PRECISION_DATA_TYPE time, PRECISION_DATA_TYPE distance, PRECISION_DATA_TYPE wheel_distance);
public:
    Ramp(PRECISION_DATA_TYPE acc, PRECISION_DATA_TYPE max_speed, PRECISION_DATA_TYPE dec, PRECISION_DATA_TYPE distance, PRECISION_DATA_TYPE initial_speed=0.0f, PRECISION_DATA_TYPE end_speed=0.0f);

    void start(PRECISION_DATA_TYPE wheel_distance);

    const RampReturnData compute(PRECISION_DATA_TYPE distance_to_point, PRECISION_DATA_TYPE wheel_distance);

    const RampReturnData compute();

};


#endif //CODEPAMI_RAMP_H
