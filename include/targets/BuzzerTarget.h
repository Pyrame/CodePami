#pragma once
#include "Target.h"

#include <Romi32U4Buzzer.h>
class BuzzerTarget : public Target{
public:

    BuzzerTarget(Robot* robot) : Target(robot){

    }

    void on_done() override{
        
    }

    void process() override{ // boucle infinie à la fin de la trajectoire tant qu'on éteint pas entièrement le robot
        PololuBuzzer buzzer;
        static bool playing = false;
        if (!playing){
            // Pirate des caraibes
            //buzzer.play("t200v80dd8dd8dv90l8ddddd4dd4dd4ddddv100d4dd4dd4dd<a>cd4d4def4f4fge4e4dccd4r<a>cd4d4def4f4fge4e4dcd4r4<a>cd4d4dfg4g4gaa+4a+4agad4rdef4f4g4ad4rdfe4e4fder4r8a>cd4d4def4f4fge4e4dccd4r<a>cd4d4def4f4fge4e4dcd4r4<a>cd4d4dfg4g4gaa+4a+4agad4rdef4f4g4ad4rdfe4e4dcl4d,t200r1r1r1o3d2.d.d.>dd8dc8<a+a+8a+a+8aa8aa8>dd8dd8<a+a+8a+a+8ff8ff8>cc8<aa8>dd8dd8dd8dd8<a+a+8a+a+8gg8gg8>dd8dd8<a+a+8a+a+8>dd8dd8<aa8aa8aa8aa8>dd8dc8<a+a+8a+a+8aa8aa8>dd8dd8<a+a+8a+a+8ff8ff8>cc8<aa8>dd8dd8dd8dd8<a+a+8a+a+8gg8gg8>>f8r2r8<a+a+a+>f8r2r8<aa,t200r1r1r1o2d2.d.d.>>fff8a8a+a+a+8>d8<aaa8g8a8ar4r8ffa+8a+8aa>c8c8cc<a8r8fr2ffa8a8a+a+>d8d8ddl8fefr2r<a+4a+4a+4>fr2r<a4a4>d<bar2r>f4f4faa+4a+4a+>d<a4a4agal4ar4r8ffa+8a+8aa>c8c8cc<a8r8fr1r1r2r<dd8dd8<a+a+8a+a+8>dd8dd8<aa8aa8");
            // Volare
            buzzer.play("t440 o4 l1 g# l2 f#. l4 c# r1 r1 l1 g#f# r1 r1 f# l2 e. l4 <b r1 l2 <bd#f#e r1 r1");
            playing = true;
        }
        if (!buzzer.isPlaying()){
            playing = false;
        }
    }

    bool is_done() override{
        return false;
    }

    void init() override{
        
    }

};