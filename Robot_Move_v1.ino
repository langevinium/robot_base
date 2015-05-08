// Code to move a robot.
//
// Requires the Adafruit_Motorshield v2 library 
//   https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
// And AccelStepper with AFMotor support 
//   https://github.com/adafruit/AccelStepper

// This tutorial is for Adafruit Motorshield v2 only!
// Will not work with v1 shields

// TODO
//  * [x] Fonction avancer/reculer
//  * [x] Fonction tourner
//  * [_] Fonction controle précis

#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

/* SECTION ZERO − DEFINITION OF FLAGS
 */

#define STATE_RUNNING  0
#define STATE_STOP 1

#define P_GO 0
#define P_TURN 1
#define P_WAIT 2

/* SECTION ONE − DESCRIPTION OF THE ROBOT AND OF THE PATH
 This section is used to define the characteristics of the robot
 and to describe its path. This section is typicaly twisted during the competition.
 */

// Define characteristics of the robot
const int stepperLength = 200;         // Number of steps per revolution
const float circumference = 3.14*5.2/2;  // Circumference of wheel (in cm)
                                       // If steppers are in DOUBLE precision, the 
                                       // circumference must be divided by two.
const float width = 12.0;              // Distance between the two wheels in cm

const float max_speed = 200.0;
const float max_acc = 100.0;

// Define the path
// /!\ Don't forget to update path_length /!\
//     {move, turn, wait}
const int path_length = 4;

const float path[][3] = {
  { 
    10, 0, 0   }
  ,    // step 0: move 10cm
  { 
    0, 0, 2000   }
  ,  // step 1: wait 2s
  { 
    20, 0, 0   }
  ,    // step 2: move 20 cm
  { 
    0, 90, 0  }
    // step 3: turn 90°
};

// ---------------------------------------------------------

/* SECTION TWO − INITIALIZATION OF SOME PARAMETERS
 In this section various parameters are initialized :
 - constants;
 - variables that describe the status of the robot;
 - variables that describe the current position in the path.
 */

// Define mathematical constants
const float pi = 3.14159;

// Define pin's aliases
const int stopPin = 8;
const int tirette = 11;
const int led_ready = 12;
const int led_standby = 13;

// Define some variables
int tirette_state = 1;            // Will store the state of the tirette.
int robot_state = STATE_RUNNING;  // Will store the state of the robot.
int step_current = 0;             // Will store the id of the current step.
int step_isFinished = true;       // Turn to true when the step is finisehd.
int step_isInited = false;        // Turn to true when the step is initialized.

unsigned long currentMillis = 0;  // Will store the current time.
unsigned long previousMillis = 0; // Will store last timer a timer was launched.
unsigned long interval = 0;       // Will store the interval of timers.


/* SECTION THREE − INITIALIZATION OF STEPPERS
 Init the shield and the two steppers.
 */

// Define the Adafruit MotorShield
Adafruit_MotorShield AFM(0x60); // Default address, no jumpers

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the shield
Adafruit_StepperMotor *myStepperRight = AFM.getStepper(stepperLength, 1);
Adafruit_StepperMotor *myStepperLeft  = AFM.getStepper(stepperLength, 2);

// Define wrappers for each motor
// You can change these to SINGLE or DOUBBLE or INTERLEAVE or MICROSTEP
void forwardStepRight() {
  myStepperRight->onestep(BACKWARD, DOUBLE);
}

void backwardStepRight() {
  myStepperRight->onestep(FORWARD, DOUBLE);
}

void forwardStepLeft() {
  myStepperLeft->onestep(FORWARD, DOUBLE);
}

void backwardStepLeft() {
  myStepperLeft->onestep(BACKWARD, DOUBLE);
}

// Wrap the 2 steppers in an AccelStepper object
AccelStepper stepperRight(forwardStepRight, backwardStepRight);
AccelStepper stepperLeft(forwardStepLeft, backwardStepLeft);


/* SECTION FOUR − SETUP
 */

void setup()
{
  // Init the serial connexion
  Serial.begin(9600);

  // Init pins
  pinMode(stopPin, INPUT);
  pinMode(tirette, INPUT_PULLUP);
  pinMode(led_ready, OUTPUT);
  pinMode(led_standby, OUTPUT);
  
  // Turn on the standby led
  digitalWrite(led_standby, HIGH);

  AFM.begin(); // Start the shield

  stepperRight.setMaxSpeed(max_speed);
  stepperRight.setAcceleration(max_acc);

  stepperLeft.setMaxSpeed(max_speed);
  stepperLeft.setAcceleration(max_acc);
  
  // Stay in a infinit loop while the tirette is missing
  while (tirette_state == HIGH) {
    tirette_state = digitalRead(tirette);
  }
  
  // The robot is ready
  // Turn on the ready led
  digitalWrite(led_ready, HIGH);
  
  // Stay in a infinit loop until the tirette is pulled
  while (tirette_state == LOW) {
    tirette_state = digitalRead(tirette);
  }
  
}

/* SECTION FIVE − MAIN LOOP
 */

void loop()
{
  // Actions in case of a urgent stop
  if (digitalRead(stopPin) == 1) {
    stop_emergency();
  }

  if (robot_state == STATE_RUNNING)
  {
    // Update the timer
    currentMillis = millis();

    // If the last step is finished
    // and if the next step is not initialized
    // init it.
    if (step_isFinished && !step_isInited) {
      // Read in the path table how to initialize the step
      if (path[step_current][P_GO] != 0)
        go(path[step_current][P_GO]);
      else if (path[step_current][P_TURN] != 0)
        turn(path[step_current][P_TURN]);
      else if (path[step_current][P_WAIT] != 0) {
        wait(path[step_current][P_WAIT]);
      }

      step_isInited = true;
      step_isFinished = false;
    }

    // Run steppers
    stepperRight.run();
    stepperLeft.run();

    // If the current step is finished, end it.
    if ( (stepperRight.distanceToGo() == 0)
      && (stepperLeft.distanceToGo() == 0)
      && (currentMillis - previousMillis > interval) ) {
      step_isFinished = true;
      step_current = step_current + 1;

      // Test if the end of the path is reached
      if (step_current == path_length) {
        // Stop steppers
        stop_normal();
      } 
      else {
        // Continue
        step_isInited = false;
      }
    }

  }
}

/* SECTION SIX − MOVING FUNCTIONS
 */

// GO
//  Move the robot forward or backward in straight line
//   - dist: distance to move (in cm)
void go(float dist_inCm)
{
  int dist_inStep = (dist_inCm * stepperLength)/(2*circumference);

  Serial.print("Go:\t");
  Serial.print(dist_inCm);
  Serial.print(" cm\t->\t");
  Serial.println(dist_inStep);

  stepperRight.move(dist_inStep);
  stepperLeft.move(dist_inStep);
}

// TURN
//  Turn the robot
//   - angle: couterclockwise angle in °
void turn(float angle)
{
  int dist_inStep = (angle * pi * width * stepperLength)/(360*circumference);

  Serial.print("Turn:\t");
  Serial.print(angle);
  Serial.print(" deg\t->\t");
  Serial.println(dist_inStep);

  stepperRight.move(dist_inStep);
  stepperLeft.move(-dist_inStep);
}

// WAIT
//  Do a break
//   - t: duration of the break (in ms)
void wait(int t)
{
  Serial.print("Wait:\t");
  Serial.print(t);
  Serial.println(" ms");

  // Launch the timer
  interval = t;
  previousMillis = currentMillis;
}

/* SECTION SEVEN − STOPING FUNCTIONS */

// Emergency STOP
//  Stop steppers and release them
void stop_emergency()
{
  Serial.println("STOP (emergency)");
  
  // Turn down the ready led
  digitalWrite(led_ready, LOW);

  robot_state = STATE_STOP;
  myStepperRight->release();
  myStepperLeft->release();

  stepperRight.disableOutputs();
  stepperLeft.disableOutputs();
}

// Normal STOP
//  Only stop steppers but don't release them.
void stop_normal()
{
  Serial.println("STOP (normal)");
  
  // Turn down the ready led
  digitalWrite(led_ready, LOW);

  robot_state = STATE_STOP;
  myStepperRight->release();
  myStepperLeft->release();

  stepperRight.disableOutputs();
  stepperLeft.disableOutputs();
}

