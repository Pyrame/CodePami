#include "Arduino.h"
#include "Position.h"
#include "Ramp.h"

#ifndef TARGET_H
#define TARGET_H
class Robot;
class Target{
protected:
    Robot* robot;
    bool is_init = false;
public:
    Target(Robot* robot);
    virtual void init() = 0;
    void call_init(){
        if(!is_init){
            init();
            is_init = true;
        }
    }
    
    virtual void reinitRamp(){
        
    }

    virtual bool is_done() = 0;
    virtual void process() = 0;

    virtual void on_done() = 0;

    virtual ~Target() = default;

};



#endif