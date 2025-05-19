#include <Arduino.h>
#include "config.h"
#include "Position.h"
#include "Robot.h"

//-D DEBUG_TXT
#include <Wire.h> // Pour Capteur ultrason

//#define TRACK_MM 142.0f *3645/3600
#define TRACK_MM 52.3f//140.2469135802469f // Entraxe // Attention a bien modifier le PID selon
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


//Servo myServo; // Create a Servo object: Faire la fete

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
uint8_t pulseWidth = 1500;  // Largeur d'impulsion en microsecondes (90° par défaut)

// Pins boutons
const int boutonA = 14; //IO_0
const int boutonB = 30; //IO_1
const int boutonC = 17; //IO_2

uint8_t pos = 0;

bool Obstacle = false;

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
const unsigned long LimitTime = 15 * 1000;  // Fin de match: 100 seconds

// Capteur Ultrason
uint8_t distlim1 = 15; // cm
uint8_t distlim2 = 5;

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
// double Bleu = 1; // A ne pas oublier pour les couleurs // Problemes a regler

void setup() {
    Serial.begin(9600);

    delay(1000);
    Serial.println("Welcome to the little PAMI! My little Pamy!");
    Wire.begin(); // Capteur Ultrason

    //myServo.attach(SERVO_PIN); // Connect the signal wire to pin 5 (or another PWM pin): Faire la fete // A modifier le PIN

    //pinMode(SWITCH_PIN, INPUT_PULLUP); // Active la résistance de pull-up interne du micro Superstar 

    // left_motor->setPWM(1000); // Pour tester les moteurs
    // right_motor->setPWM(1000);


    Position init_pos(-1000,0,0); // Position initiale 
    Position pos(1000,1000,80); 
    
    robot = new Robot(left_motor,right_motor, TICK_PER_MM, TRACK_MM, 1.0f); // Tick
    robot->setPidDistance(new PID(3, 0.1, 0.02)); // PID settings // ROMI: 3, 0.1, 0.02 // A regler en fonction du PAMI
    robot->setPidAngle(new PID(4, 0.02, 0.01)); // ROMI: 2, 0.02, 0.01
    
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
    // Trajectoires
    #ifdef GOUPILLE_ENABLED
    if (stratChoice == 0){
    #else
    if (digitalRead(boutonA) == LOW){ //&& digitalRead(goupille) == HIGH){
    #endif
        // Serial.println("Bouton A pressé: Strat Bleue engagée");
        // Bleu = -1;
        Serial.println("Bouton A pressé");
        delay(4000); // 4 secondes - Simule l'attente apres demarrage de match (normalement 85 sec)
        robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1000, -300, -45},100)); // Strat Loin Jaune
        robot->addTarget(new BezierTarget(robot, 300, 400, 300, {1900, -250, 45},1000));
    }
    #ifdef GOUPILLE_ENABLED
    if (stratChoice == 1){
    #else
    if (digitalRead(boutonB) == LOW){ //&& digitalRead(goupille) == HIGH){
    #endif
        Serial.println("Bouton B pressé");
        delay(4000);
        robot->addTarget(new BezierTarget(robot, 300, 400, 300,{700, -300, 45}, 100)); // Strat Milieu Jaune // Test V1 PAMI: parfait
        robot->addTarget(new BezierTarget(robot, 300, 400, 300, {1400, -200, 45},100));
        // robot->addTarget(new PositionTarget(robot, {2000,0}, 300,400,300)); // Strat Superstar Jaune de merde: Ne jamais utiliser!!!!
        // robot->addTarget(new PositionTarget(robot, {1200,-300}, 300,400,300));
    }
    // if (digitalRead(boutonC)== LOW && digitalRead(goupille) == HIGH){
    //     Serial.println("Bouton C presse");
    //     delay(4000);
    //     robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1000, -170, -45}, 1000));// Strat Proche Jaune
    // }
    #ifdef GOUPILLE_ENABLED
    if (stratChoice == 2){
    #else
    if (digitalRead(boutonC) == LOW){ //&& digitalRead(goupille) == HIGH){
    #endif
        Serial.println("Bouton C presse");
        delay(4000);
        robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1200, 10, 0}, 100)); // Strat Superstar Jaune
        robot->addTarget(new BezierTarget(robot, 300, 400, 300,{1200, -375, 0}, 100));
    }
    //robot->addTarget(new BuzzerTarget(robot)); // simulation de l'actionneur a retirer quand l'actionneur arrive //Faire la fete (oui ca reste pour si la tache a ete faite correctement) ////////////////////////// Voir si l'actionneur se met en route seulement a la fin du match sinon il faudra preciser de ne pas actionner tant que le match n'est pas fini mais je pense qu'on est bon si on met que dans le loop() comme dans l'etat actuel
    
    
    //robot->addTarget(new PositionTarget(robot, {1200,0}, 300,400,300)); // Strat Superstar Jaune de merde: Ne jamais utiliser!!!!
    //robot->addTarget(new PositionTarget(robot, {1200,-300}, 300,400,300));

    // robot->addTarget(new RotateTowardPositionTarget(robot, {0,1000}));
    // robot->addTarget(new PositionTarget(robot, {0,1000}, 300,400,300));
    // robot->addTarget(new RotateTowardPositionTarget(robot, {0,0}));
    // robot->addTarget(new PositionTarget(robot, {0,0}, 300, 400, 300));

    previous_micros = micros();

    //robot->addTarget(new AngleTarget(robot, 360));
    //robot->addTarget(new AngleTarget(robot, -170));
    //robot->addTarget(new RotateTowardPositionTarget(robot, pos));
}

uint8_t current_count = 1;

void loop() {
    while(micros() - previous_micros < 5000); // Pour eviter que le code tourne tout le temps (pour la memoire)
    if(millis()- StartingTime >= LimitTime){
        left_motor->setPWM(0);
        right_motor->setPWM(0);
        Serial.println("Sorry for Party Rockin!");
        //while(true);
        while(true){
            // Met à jour la largeur d'impulsion en fonction d'une valeur quelconque
            int angle = analogRead(A0) / 5.7;  // Exemple : Lire un potentiomètre sur A0
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
    if(current_count%16 == 0){ // 15 a la base mais 16 fait gagner 0.2% de memoire 
        int distance = readDistance();
        if (distance >= 0) {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" cm");
            // if(distance <= distlim1 && distance > distlim2){ // Obstacle
            //     left_motor->setPWM(30);
            //     right_motor->setPWM(30);
            // }
            if(distance <= distlim1){
                Obstacle = true;
                robot->setRampSpeed(0);
                robot->setRampSpeedAngle(0);
                left_motor->setPWM(0);
                right_motor->setPWM(0); 
            }
            else{ 
                if(Obstacle){
                    robot->resetTarget();   
                }
                Obstacle = false;
            }
        } else {
            Serial.println("Error reading distance");
        }
        delayMicroseconds(100); 
        startMeasurement();
    }
    current_count++;
    robot->computePosition(Romi32U4Encoders::getCountsAndResetLeft(), Romi32U4Encoders::getCountsAndResetRight());
    robot->computeTarget();
    robot->control();

    #ifdef DEBUG_TXT
    //Serial.println(*robot);
    #endif
    if (Obstacle){
        left_motor ->setPWM(0);
        right_motor -> setPWM(0);
    }

    //Serial.println(freeMemory());
    }


// Ne pas oublier le dipswitch

// Ne pas oublier de modifier les PINS

// Verifier si on part bien d'une position definie sur le repere ou si il croit tjrs qu'on le pose en 0,0,0: Il croit qu'on est tjrs en 0,0,0. donc attention
// Ne pas oublier que les PAMI ne sont pas toujours placés au meme endroit! Note: plus on est loin du bord, plus on va loin 
// Dans le cas ou il pense qu'il est en 0,0,0 Il faut: faire attention a l'ordre des PAMI et au case choisi! ET/OU definir une position de depart pour chaque strat (mais la encore il faut faire attention au positionnement)
// Mot d'ordre: Attention aux positionnements des PAMI et du case choisi : Numéroter les PAMIs (ecrire dessus la position qu'il doit avoir plutot que de juste numeroter)
// Normalement, chaque PAMI a sa propre calibration (son propre PID) (de nouveau, bien numeroter)