#include <Arduino.h>
#include "config.h"
#include "Position.h"
#include "Robot.h"
#include <Romi32U4.h>

#include <Wire.h> // Pour Capteur ultrason

//#define TRACK_MM 142.0f *3645/3600
#define TRACK_MM 140.2469135802469f
#define TICK_PER_MM 6.58f
#define Pin_PWM_Left 10 //PWM
#define Pin_Dir_Left 16
#define Pin_PWM_Right 9
#define Pin_Dir_Right 15

#define SRF08_ADDRESS 0xE0 >> 1  // Default address (0xE0 shifted for Arduino Wire library) // Pour Capteur ultrason
#define COMMAND_REGISTER 0x00
#define RANGE_HIGH_BYTE 0x02

#define SWITCH_PIN 1  // Broche du microswitch

#include "Ramp.h"
#include "targets/BezierTarget.h"
#include "targets/PositionTarget.h"
#include "targets/RotateTowardPositionTarget.h"
#include "targets/AngleTarget.h"

//#include <Romi32U4Buzzer.h>

// Renaming 
//PololuBuzzer buzzer;
Romi32U4Motors motors; 
Robot* robot;
Motor* left_motor = new Motor(Pin_Dir_Left, Pin_PWM_Left);
Motor* right_motor = new Motor(Pin_Dir_Right, Pin_PWM_Right);

// Pins boutons
const int boutonA = 14; //IO_0
const int boutonB = 30; //IO_1
const int boutonC = 17; //IO_2

//uint64_t last_time;

// Memory 
int freeMemory() {
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// Timing
uint64_t previous_micros;
unsigned long StartingTime;  
const unsigned long LimitTime = 100 * 1000;  // Fin de match: 100 seconds

// Capteur Ultrason
uint64_t distlim1 = 10; // cm
uint64_t distlim2 = 5;

void startMeasurement() {
    Wire.beginTransmission(SRF08_ADDRESS);
    Wire.write(COMMAND_REGISTER);
    Wire.write(0x51);  // Command for range in cm
    Wire.endTransmission();
}

int readDistance() {
    Wire.beginTransmission(SRF08_ADDRESS);
    Wire.write(RANGE_HIGH_BYTE);
    Wire.endTransmission();
    delayMicroseconds(100);
    Wire.requestFrom(SRF08_ADDRESS, 2);
    while(Wire.available() < 2);    
    if (Wire.available() >= 2) {
        int highByte = Wire.read();
        int lowByte = Wire.read();
        return (highByte << 8) | lowByte;
    }
    return -1;  // Error
}

void setup() {
    Serial.begin(9600);

    Wire.begin(); // Capteur Ultrason

    pinMode(SWITCH_PIN, INPUT_PULLUP); // Active la résistance de pull-up interne

    Position init_pos(-1000,0,0); // Position initiale
    Position pos(1000,1000,80); 
    
    robot = new Robot(left_motor,right_motor, TICK_PER_MM, TRACK_MM, 1.0f); // Tick
    robot->setPidDistance(new PID(3, 0.1, 0.02)); // PID settings
    robot->setPidAngle(new PID(2, 0.02, 0.01));
    //robot->addTarget(new AngleTarget(robot, 360*40, 90,90,360,false));

    //robot->addTarget(new PositionTarget(robot, init_pos, 300,300,400,300));
    //robot->addTarget(new BezierTarget(robot, 300, 400, 300, pos, 3000));
    //robot->addTarget(new BezierTarget(robot, 300,400,300,{2000, 1000, -80}, 1000));
    //robot->addTarget(new BezierTarget(robot, 300,400,300,{3000, 1000, 80}, 1000));
    //robot->addTarget(new AngleTarget(robot, 0));

    //robot->addTarget(new PositionTarget(robot, {300, 0}, 300, 400, 300));

    pinMode(boutonA, INPUT_PULLUP); // Boutons // Remarque: peut etre qu'on pourrait fonctionner de la meme maniere avec le dipswitch: 1 pin 1 strat
    pinMode(boutonB, INPUT_PULLUP);
    pinMode(boutonC, INPUT_PULLUP);
    while (!digitalRead(boutonA) == LOW && !digitalRead(boutonB) == LOW && !digitalRead(boutonC) == LOW) // Init boutons

    StartingTime = millis();  // Start the timer 

    // Trajectoires
    if (digitalRead(boutonA) == LOW){
        delay(4000); // 4 secondes - Simule l'attente apres demarrage de match (normalement 85 sec)
        robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1000, -400, -45}, 100)); // Strat Loin Jaune
        robot->addTarget(new BezierTarget(robot, 300, 400, 300, {1900, -170, 45},1000));
    }
    if (digitalRead(boutonB) == LOW){
        delay(4000);
        robot->addTarget(new BezierTarget(robot, 300, 400, 300,{700, -300, -45}, 100)); // Strat Milieu Jaune
        robot->addTarget(new BezierTarget(robot, 300, 400, 300, {1400, -170, 45},1000));
    }
    if (digitalRead(boutonC) == LOW){
        delay(4000);
        robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1000, -180, -30}, 100));// Strat Proche Jaune
    }
    //robot->addTarget(new BuzzerTarget(robot)); // simulation de l'actionneur a retirer quand l'actionneur arrive //Faire la fete (oui ca reste pour si la tache a ete faite correctement) ////////////////////////// Voir si l'actionneur se met en route seulement a la fin du match sinon il faudra preciser de ne pas actionner tant que le match n'est pas fini mais je pense qu'on est bon si on met que dans le loop() comme dans l'etat actuel
    
    
    //robot->addTarget(new PositionTarget(robot, {1200,0}, 300,400,300)); // Strat Superstar Jaune
    //robot->addTarget(new PositionTarget(robot, {1200,-300}, 300,400,300));


    //robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1000, 400, 45}, 100)); // Strat Loin Bleu
    //robot->addTarget(new BezierTarget(robot, 300, 400, 300, {1900, 170, -45},1000));

    //robot->addTarget(new BezierTarget(robot, 300, 400, 300,{700, 300, 45}, 100)); // Strat Milieu Bleu
    //robot->addTarget(new BezierTarget(robot, 300, 400, 300, {1400, 170, -45},1000));

    //robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1000, 180, 30}, 100)); // Strat Proche Bleu

    //robot->addTarget(new PositionTarget(robot, {1200,0}, 300,400,300)); // Strat Superstar Bleu
    //robot->addTarget(new PositionTarget(robot, {1200,300}, 300,400,300));

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

int current_count = 1;

void loop() {
    while(micros() - previous_micros < 5000); // Pour eviter que le code tourne tout le temps (pour la memoire)
    if(millis()- StartingTime >= LimitTime){
        motors.setSpeeds(0, 0); // Stop the motors if the match has ended
        Serial.println("Match Over. PAMI Stopped.");
        // while(true){
        //     //buzzer.play("t440 o4 l1 g# l2 f#. l4 c# r1 r1 l1 g#f# r1 r1 f# l2 e. l4 <b r1 l2 <bd#f#e r1 r1"); // Il faudra mettre la boucle de l'actionneur ici, comme ca on est sur
        //     delay(8000); // pour permettre a la chanson de se faire entierement 
        // };
    }
    previous_micros = micros();
    delayMicroseconds(5000);
    robot->computePosition(Romi32U4Encoders::getCountsAndResetLeft(), Romi32U4Encoders::getCountsAndResetRight());
    robot->computeTarget();
    robot->control();
    //uint64_t next = micros();
    //Serial.println(((double)(next-last_time))/1000.0f);
    //last_time = next;

    // Lecture distance et Obstacle
    if(current_count%15 == 0){
        int distance = readDistance();
        if (distance >= 0) {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" cm");
            if(distance <= distlim1 && distance > distlim2){ // Obstacle
                motors.setSpeeds(1,1);
            }
            if(distance <= distlim2){
                motors.setSpeeds(0,0);
            }
        } else {
            Serial.println("Error reading distance");
        }
        delayMicroseconds(100);
        startMeasurement();
    }
    current_count++;
    #ifdef DEBUG_TXT
    Serial.println(*robot);
    #endif

    // Superstar Microswitch
    // if (digitalRead(SWITCH_PIN) == HIGH) {  // Si le switch est relâché
    //     motors.setSpeeds(0, 0);  // Arrêter le robot
    // }


    //Serial.println(freeMemory());
    }

// Ne pas oublier la goupille!!!!

