#include "Arduino.h"
#include "config.h"
#ifndef ANTI_WINDUP_VALUE
    #define ANTI_WINDUP_VALUE 50
#endif
#ifndef PID_H
#define PID_H
class PID{
   PRECISION_DATA_TYPE kp;
   PRECISION_DATA_TYPE ki;
   PRECISION_DATA_TYPE kd;

   PRECISION_DATA_TYPE old_error;
   PRECISION_DATA_TYPE iTerm;
   PRECISION_DATA_TYPE (*anti_windup)(PRECISION_DATA_TYPE);

   PRECISION_DATA_TYPE previous_time;
  public:
   PID(PRECISION_DATA_TYPE kp, PRECISION_DATA_TYPE ki, PRECISION_DATA_TYPE kd, PRECISION_DATA_TYPE (*anti_windup)(PRECISION_DATA_TYPE)= [](PRECISION_DATA_TYPE a){return max(min(a, ANTI_WINDUP_VALUE), -ANTI_WINDUP_VALUE);});

   PRECISION_DATA_TYPE evaluate(PRECISION_DATA_TYPE error);

   void resetTerms();
//    PRECISION_DATA_TYPE resetPID(PRECISION_DATA_TYPE error);
};

#endif