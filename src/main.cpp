#include <Arduino.h>
#include "config.h"
#include "Position.h"
#include "Robot.h"

//-D DEBUG_TXT
#include <Wire.h> // Pour Capteur ultrason


//#define TRACK_MM 142.0f *3645/3600
#define PAMI 2 // Pami numéro: 1,2,3,4
#define Capteur 1 // 20, 0, 15, 25 cm (25 pour Homologation)
#define SUPERSTARALT 1 // 1: Normal // 2: Escalier

#define TRACK_MM 47.1f// (1) //52.3f // Proto 1 //140.2469135802469f // Entraxe // Attention a bien modifier le PID selon
#define TICK_PER_MM 6.58f
#define Pin_PWM_Left 10 //PWM
#define Pin_Dir_Left 16
#define Pin_PWM_Right 9
#define Pin_Dir_Right 15
#define GOUPILLE 12
#define GOUPILLE_ENABLED
#define SERVO_PIN 11  // Pin où est connecté le servo

#define SRF08_ADDRESS 0xE0 >> 1  // Default address (0xE0 shifted for Arduino Wire library) // Pour Capteur ultrason
#define COMMAND_REGISTER 0x00
#define RANGE_HIGH_BYTE 0x02


#include "Ramp.h"
#include "targets/BezierTarget.h"
#include "targets/PositionTarget.h"
#include "targets/RotateTowardPositionTarget.h"
#include "targets/AngleTarget.h"
#include <Romi32U4.h>
//#include <Romi32U4Buzzer.h>

// Renaming 
//PololuBuzzer buzzer;
Romi32U4Motors motors; 
Robot* robot;
Motor* left_motor = new Motor(Pin_Dir_Left, Pin_PWM_Left);
Motor* right_motor = new Motor(Pin_Dir_Right, Pin_PWM_Right);

//Servo
unsigned long previousMicros = 0;  // Stocke le dernier temps où l'impulsion a commencé
uint16_t pulseWidth = 1500;  // Largeur d'impulsion en microsecondes (90° par défaut)

// Pins boutons
const int boutonA = 14; //IO_0
const int boutonB = 30; //IO_1
const int boutonC = 17; //IO_2

// uint8_t pos = 0;

bool Obstacle = false;
bool Vu = true;

// bool poscal = false;

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
const unsigned long LimitTime = 100000;  // Fin de match: 100 seconds

// Capteur Ultrason

#if Capteur == 1 // Classique
    uint8_t distlim1 = 20; // cm
#endif
#if Capteur == 2 // Superstar
    uint8_t distlim1 = 0;
#endif
#if Capteur == 3
    uint8_t distlim1 = 15;
#endif
#if Capteur == 4 // Homologation
    uint8_t distlim1 = 25;
#endif

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


uint8_t buttonA = 0;
uint8_t buttonB = 0;
uint8_t buttonC = 0;

uint8_t previousStateA = 0;
uint8_t previousStateB = 0;
uint8_t previousStateC = 0;


void setup() {
    Serial.begin(9600);
    Romi32U4Motors::allowTurbo(true);

    delay(1000);
    Serial.println("Welcome to the little PAMI! My little Pamy!");
    Wire.begin(); // Capteur Ultrason

    
    robot = new Robot(left_motor,right_motor, TICK_PER_MM, TRACK_MM, 1.002f); // Tick // 1.005f (1)
    robot->setPidDistance(new PID(6, 7, 0.2)); // V2 : 3 0.1 0.02// PID settings // ROMI: 3, 0.1, 0.02 // A regler en fonction du PAMI
    robot->setPidAngle(new PID(6, 7, 0.2)); // V2: 7 0.02 0.027// ROMI: 2, 0.02, 0.01

    // left_motor->setPWM(1000); // Pour tester les moteurs
    // right_motor->setPWM(1000);
    
    //robot->addTarget(new AngleTarget(robot, 360*40, 90,90,360,false)); // Template AngleTarget
    //robot->addTarget(new AngleTarget(robot, 0));


    pinMode(boutonA, INPUT_PULLUP); // Boutons // Remarque: peut etre qu'on pourrait fonctionner de la meme maniere avec le dipswitch: 1 pin 1 strat
    pinMode(boutonB, INPUT_PULLUP);
    pinMode(boutonC, INPUT_PULLUP);

    pinMode(SERVO_PIN, OUTPUT); // Servo-moteur fete

    #ifdef GOUPILLE_ENABLED
    pinMode(GOUPILLE, INPUT_PULLUP); // Goupille
    #endif
    #ifdef GOUPILLE_ENABLED
    while(digitalRead(GOUPILLE) == LOW);
    delay(100);
    while(digitalRead(GOUPILLE) == HIGH){
        uint8_t stateA = !digitalRead(boutonA);
        uint8_t stateB = !digitalRead(boutonB);
        uint8_t stateC = !digitalRead(boutonC);
        
        buttonA ^= stateA && stateA != previousStateA;
        buttonB ^= stateB && stateB != previousStateB;
        buttonC ^= stateC && stateC != previousStateC;


        previousStateA = stateA;
        previousStateB = stateB;
        previousStateC = stateC;
    }
    Serial.println(buttonA);
    Serial.println(buttonB);
    Serial.println(buttonC);
    #endif
    #ifdef GOUPILLE_ENABLED
    #else
    while (!digitalRead(boutonA) == LOW && !digitalRead(boutonB) == LOW && !digitalRead(boutonC) == LOW); //&& !digitalRead(goupille) == HIGH); // Init boutons
    #endif
    StartingTime = millis();  // Start the timer 
    #ifdef GOUPILLE_ENABLED
    uint8_t stratChoice = (buttonC << 1) | buttonB; 
    #endif


    ///////////////// Trajectoires /////////////////////

    /////////////////////////////
    // Jaune
    /////////////////////////////

    #ifdef GOUPILLE_ENABLED
    if (stratChoice == 0){
    #else
    if (digitalRead(boutonA) == LOW){ //&& digitalRead(goupille) == HIGH){
    #endif
        //Serial.println("Rien de pressé: Strat Jaune");
        delay(4000); // Attente apres demarrage de match (normalement 85 sec)


        // robot->addTarget(new PositionTarget(robot, {2000,0}, 300,400,300)); // Test PID

        // Strat Loin Jaune 1
        #if PAMI == 1
            robot->addTarget(new BezierTarget(robot, 200, 400, 200,{1000, -400, -55},1000)); 
            robot->addTarget(new BezierTarget(robot, 200, 400, 300, {1900, -200, 35},1000));
        #endif

        // Strat Milieu Jaune 2
        #if PAMI == 2
            delay(2000);
            robot->addTarget(new RotateTowardPositionTarget(robot, {1500,-400}));
            robot->addTarget(new PositionTarget(robot, {1500,-400}, 300,300,400));
        #endif

        // Strat Proche Jaune 3
        #if PAMI == 3
            delay(4000);
            robot->addTarget(new RotateTowardPositionTarget(robot, {1000,-400}));
            robot->addTarget(new PositionTarget(robot, {1000,-400}, 300,300,400));
        #endif

        //Superstar Jaune 4
        #if PAMI == 4
            #if SUPERSTARALT == 1
            robot->addTarget(new PositionTarget(robot, {540,0}, 100,300,400)); //

            robot->addTarget(new PositionTarget(robot, {1200,0}, 100,300,400)); // Commence Strat a Lolo
            robot->addTarget(new RotateTowardPositionTarget(robot, {1220,-285})); // Pour s'assurer qu'il fasse le tour dans le bon sens
            robot->addTarget(new RotateTowardPositionTarget(robot, {1200,-285}));
            robot->addTarget(new PositionTarget(robot, {1200,-285}, 300,300,400));
            robot->addTarget(new RotateTowardPositionTarget(robot, {1800,-285}));
            # endif
            #if SUPERSTARALT == 2
            // robot->addTarget(new PositionTarget(robot, {500,0}, 100,300,400)); //
            robot->addTarget(new PositionTarget(robot, {1200,0}, 300,300,400)); // Commence Strat a Lolo
            robot->addTarget(new RotateTowardPositionTarget(robot, {1220,-285})); // Pour s'assurer qu'il fasse le tour dans le bon sens
            robot->addTarget(new RotateTowardPositionTarget(robot, {1200,-285}));
            robot->addTarget(new PositionTarget(robot, {1200,-285}, 300,300,400));
            robot->addTarget(new RotateTowardPositionTarget(robot, {1800,-285}));
            # endif
        #endif
    }


    /////////////////////////////
    // Bleue
    /////////////////////////////


    #ifdef GOUPILLE_ENABLED
    if (stratChoice == 1){
    #else
    if (digitalRead(boutonB) == LOW){ //&& digitalRead(goupille) == HIGH){
    #endif
        //Serial.println("Bouton B pressé: Strat Bleue");
        delay(4000); // Attente apres demarrage de match (normalement 85 sec)

        // Strat Loin Bleue 1
        #if PAMI == 1
            robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1000, 300, 45},1000)); 
            robot->addTarget(new BezierTarget(robot, 300, 400, 300, {2000, 100, -55},1000));
        #endif

        // Strat Milieu Bleue 2
        #if PAMI == 2
            delay(2000);
            robot->addTarget(new RotateTowardPositionTarget(robot, {1500,400}));
            robot->addTarget(new PositionTarget(robot, {1500,400}, 300,300,400));
        #endif

        // Strat Proche Bleue 3
        #if PAMI==3
            delay(4000);
            robot->addTarget(new RotateTowardPositionTarget(robot, {1000,400}));
            robot->addTarget(new PositionTarget(robot, {1000,400}, 300,300,400));
        #endif

        //Superstar Bleue 4 (Mieux ici de pas prendre la strat de Laurent a priori)
        #if PAMI == 4
            #if SUPERSTARALT == 1
            robot->addTarget(new PositionTarget(robot, {560,0}, 100,300,400));
            robot->addTarget(new PositionTarget(robot, {1200,0}, 100,300,400)); 
            robot->addTarget(new RotateTowardPositionTarget(robot, {1220,300})); // Pour s'assurer qu'il fasse le tour dans le bon sens
            robot->addTarget(new RotateTowardPositionTarget(robot, {1200,300}));
            robot->addTarget(new PositionTarget(robot, {1200,300}, 300,300,400));
            robot->addTarget(new RotateTowardPositionTarget(robot, {1800,300}));
            #endif
            #if SUPERSTARALT == 2
            // robot->addTarget(new PositionTarget(robot, {500,0}, 100,300,400));
            robot->addTarget(new PositionTarget(robot, {1200,0}, 300,300,400)); 
            robot->addTarget(new RotateTowardPositionTarget(robot, {1220,250})); // Pour s'assurer qu'il fasse le tour dans le bon sens
            robot->addTarget(new RotateTowardPositionTarget(robot, {1200,250}));
            robot->addTarget(new PositionTarget(robot, {1200,250}, 300,300,400));
            robot->addTarget(new RotateTowardPositionTarget(robot, {1800,250}));
            #endif
        #endif
    }

    #ifdef GOUPILLE_ENABLED
    if (stratChoice == 2){
    #else
    if (digitalRead(boutonC) == LOW){ //&& digitalRead(goupille) == HIGH){
    #endif
        // // Serial.println("Bouton C presse");
        // delay(85000); // Attente apres demarrage de match (normalement 85 sec)

        
    }

    startMeasurement();

    previous_micros = micros();
    
}

unsigned long current_count = 1;

void loop() {
    while(micros() - previous_micros < 5000); // Pour eviter que le code tourne tout le temps (pour la memoire)
    if(millis()- StartingTime >= LimitTime){
        left_motor->setPWM(0);
        right_motor->setPWM(0);
        Serial.println("Sorry for Party Rockin!");
        while(true){
            // Met à jour la largeur d'impulsion en fonction d'une valeur quelconque
            int angle = 180;//analogRead(A0) / 5.7;  // Exemple : Lire un potentiomètre sur A0
            pulseWidth = map(angle, 0, 180, 500, 2500);  

            // Gestion non bloquante du signal PWM
            static bool pulseState = false;
            unsigned long currentMicros = micros();


            if (!pulseState && (currentMicros - previousMicros >= 20000)) {
                // Début d'un nouveau cycle de 20ms (50Hz)
                digitalWrite(SERVO_PIN, HIGH);
                previousMicros = currentMicros;
                pulseState = true;
            }

            if (pulseState && (currentMicros - previousMicros >= pulseWidth)) {
                // Fin de l'impulsion après `pulseWidth` microsecondes
                digitalWrite(SERVO_PIN, LOW);
                pulseState = false;
            }
        };
    }

    previous_micros = micros();
    delayMicroseconds(5000);

    //Lecture distance et Obstacle
    if(current_count == 13){ // 13 et 16 tourne et s'arrete mais restart merde; // 16
        current_count = 0;
        int distance = readDistance();
        if (distance >= 0) {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" cm");
            Serial.println(*robot);
            if(distance <= distlim1){
                
                Obstacle = true;
                robot->setRampSpeed(0);
                robot->setRampSpeedAngle(0);
                left_motor->setPWM(0);
                right_motor->setPWM(0);

                /////// Nouveau pour l'évitement : Pas en Belgique /////////////////
                // Que programmé pour jaune pour le moment!!
                

                if(Vu && robot->getTargetCount() != 0){
                    //Current position
                    Position currentPos = robot->getPosition();
                    float angle = currentPos.getAngle();
                    //robot->rememberTarget(); // Retient la cible
                    //robot->clearTargets(); // Attention: Efface les targets
                    distlim1 = 0;
                    robot->injectRotateToward();

                    // Rotation de 70 deg
                    float newAngle = angle - 70; // Pas besoin d'aller jusque 90 (qui est imprevisible niveau sens), 70 suffit
                    // if (newAngle < -180) newAngle += 360;
                    float dx_forward = 150 * cos(newAngle * M_PI / 180.0);
                    float dy_forward = 150 * sin(newAngle * M_PI / 180.0);
                    Position aroundPos = {currentPos.getX() + dx_forward, currentPos.getY() + dy_forward};
                    robot->injectTarget(new PositionTarget(robot, aroundPos, 300, 300, 300), &distlim1, 20);
                    Serial.println("Avancer");
                    
                    robot->injectTarget(new AngleTarget(robot, newAngle));
                    Serial.println("Rotation");
                    // Avancer (contourner)
                    
                    // Revenir à l'angle original
                    //robot->addTarget(new AngleTarget(robot, angle));
                    Serial.println("Revenir à l'angle de base");
                    //robot->addTarget(robot->getRememberedTarget()->clone());
                    Vu = false;
                    Serial.println("Vu: Il ne fera plus d'évitement");
                    
                    
                }
                
            }
            else{ 
                if(Obstacle){
                    Serial.println("Reset ramp");
                    robot->setRampSpeed(0);
                    robot->setRampSpeedAngle(0);
                    robot->resetTarget();
                    
                }
                Obstacle = false;
                
            }
        } else {
            // Serial.println("Error reading distance");
        }
        delayMicroseconds(100); 
        startMeasurement();
    }
    current_count++;
    robot->computePosition(Romi32U4Encoders::getCountsAndResetLeft(), Romi32U4Encoders::getCountsAndResetRight());
    if(!Obstacle){
        robot->computeTarget();
        robot->control();
    }
    #ifdef DEBUG_TXT
    // Serial.println(*robot);
    #endif
    if (Obstacle){
        left_motor ->setPWM(0);
        right_motor -> setPWM(0);
    }

    //Serial.println(freeMemory());
}
