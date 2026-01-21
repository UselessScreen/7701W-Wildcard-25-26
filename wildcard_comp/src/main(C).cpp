//TODO: future additions
//TODO: add functional autonomous for skills
//TODO: fix scoring motor (possible hardware issue)
//// add extra motor for lift mechanism (PORT15)
//TODO: add brain inertial (possibly) for accurate autonomous driving
//TODO: tune turning speed
//TODO: add more comments throughout code
//TODO: add global interrupt for left button
//TODO: Relocate code to skills
//// replace "magic numbers" with defined constants where applicable

#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <string>
#include "vex.h"

using namespace vex;
using std::string;
using std::cout;
using std::endl;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS

// Robot configuration code.
// PORT 4 IS RESERVED ----- FORTNITE

// Independent motors
motor scoreMotor = motor(PORT10, ratio6_1, true);
motor intakeMotor = motor(PORT3, ratio6_1, true);

// Drivetrain motors
motor leftMotorA = motor(PORT11, ratio6_1, false);
motor leftMotorB = motor(PORT1, ratio6_1, false);
motor leftMotorC = motor(PORT19, ratio6_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT7, ratio6_1, true);
motor rightMotorB = motor(PORT20, ratio6_1, true);
motor rightMotorC = motor(PORT8, ratio6_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 11.43, 12.953999999999999, mm, 1.3333333333333333);

controller Controller1 = controller(primary);

// generating and setting random seed
void initializeRandomSeed()
{
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}




void vexcodeInit()
{

  // Initializing random seed.
  initializeRandomSeed();
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName)
{
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

volatile bool interrupt = false;

void onLeftPressed() {
  // Function to be called when the left button is pressed
  interrupt = true;
}

void controlSet() {
  // Controller setup
  Controller1.ButtonLeft.pressed(onLeftPressed);
}

// Initialise the controller task --> Controller1
task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

#pragma endregion VEXcode Generated Robot Configuration
/* #region project-information */
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp.                                                 */
/*    Author:       Team 7701W Wildcard                                       */
/*    Created:      01/11                                                     */
/*    Description: Our code, automatic 15 second autonomous period and 45     */
/*                second driver control period with skills autonomous.        */
/*                Left button for driving skills, right button for autonomous.*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
/* #endregion */

//* Constants and Globals
const float bufferS = 2.25;                   // Buffer time in seconds
const int buffer2 = bufferS * 1000 + 10;     // Buffer time in milliseconds + 10 extra for safety
const int intakeSpeed = 50;                 // Intake motor speed
const int scoreSpeed = 100;                // Scoring motor speed
const int counterSpeed = 100;             // Counter motor speed
const int turnSpeed = 50;                // Drivetrain turning speed
const int tile = 600;                   // One tile distance in mm 
const int driveSpeed = 100;            // Drivetrain speed

//* Pre-call functions for misc tasks
void screenReset() {
  // Resets the brain screen to default state
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
}

void buffer() {
  //! The legendary fake init function that does nothing
  //* but gives time to setup -- robot experiences inconsistent communication issues when removed
  timer t;
  t.reset();
  int i = 0;

  while (t.time(sec) <= bufferS) {
    //Simple loading animation just for filler
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Parthogenesis.init()");    
    string dots = string(i, '.');
    Brain.Screen.print(dots.c_str());
    i = (i + 1) % 4;
    wait(250, msec);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,21);
  }
  //Remove loading text after buffer completed
  screenReset();
}

void driveSetup() {
  // Motor setup
  Drivetrain.setDriveVelocity(driveSpeed, percent);
  Drivetrain.setTurnVelocity(turnSpeed, percent);
  intakeMotor.setVelocity(intakeSpeed, percent);
  scoreMotor.setVelocity(scoreSpeed, percent);
  //todo liftMotor.setVelocity(liftSpeed, percent);
}
 
void stopAllMotors() {
  // Stops all motors
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
  intakeMotor.stop();
  scoreMotor.stop();
  Drivetrain.stop();
}

//* Main functions for autonomous and driver control
void drive() {
  // Driver control code
  screenReset();
  interrupt = false; // reset interrupt flag
  
  Brain.Screen.print("Drive code started");
  while (true) {
    
    // Exit drive code and go to skills autonomous on RIGHT button press
    if (interrupt) {
      stopAllMotors();
      return;
    }
   
    // Scoring control
    if (Controller1.ButtonR1.pressing()) {
      intakeMotor.setVelocity(intakeSpeed, percent);
      scoreMotor.setVelocity(counterSpeed, percent);
      intakeMotor.spin(forward);
      scoreMotor.spin(reverse); // to prevent jamming while intaking

     }
    

    // Intake control
    else if (Controller1.ButtonL1.pressing()) {
      scoreMotor.setVelocity(scoreSpeed, percent);
      scoreMotor.spin(forward);


    } 
    // Stops motors in absence of input
    else {
      scoreMotor.stop();
      intakeMotor.stop();
    }

    //! Add onto ALL while loops to prevent wasted CPU cycles
    wait(20, msec);
  }
}

void matchAutonomous() {
  //  Autonomous code
  screenReset();
  // Randomized messages for autonomous start
  string msg[] = {"WIP brochacho", "otw broseph", "not yet brotation", "in dev brodude"};
  int randIndex = rand() % 4;
  Brain.Screen.print(msg[randIndex].c_str());
  // Skip autonomous if interrupt flag is set
  interrupt = false; // reset interrupt flag
    if (interrupt) {
      stopAllMotors();
      return;
      }
 
      // Autonomous actions
  Drivetrain.driveFor(forward, 500, mm);
  Drivetrain.driveFor(reverse, 500, mm); 
}




int main() {
  //* Initialize the VEXcode system DO NOT REMOVE!
  vexcodeInit();
    
  //Buffer time to allow for setup
  buffer();
  // Autonomous for 15 seconds
  timer t;
  t.reset();
  if (t.time(sec) < 15) {
    matchAutonomous();
  }
  else {
   stopAllMotors();
  }

  // Driver control forever 
  drive();

  return 0;
}
