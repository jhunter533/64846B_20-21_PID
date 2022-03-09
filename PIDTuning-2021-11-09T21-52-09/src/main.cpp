/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\CibolaA                                          */
/*    Created:      Tue Nov 09 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include "robot-config.h"
using namespace vex;

motor_group LeftDriveSmart = motor_group(FrontLeft, BackLeft);

motor_group RightDriveSmart = motor_group(FrontRight, BackRight);

smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 40, mm, 1);

//////////////PID
/// Turning////////////////////////////////////////////////////////
// PID = Porportion, Inegral, Deriviative (Tuning Parameters)
// Porportion: Distance to target angle
// Inegral: Acumulated error within the threshold
// Derivative: Rate of change of the error (Note: not needed for our purposes)

// Guidelines for Tuning:
//    - First: Baseline your parameters to learn the right values for Your
//    Robot!!
//        - set kP = 0.1 (this seems to be a good place to start)
//        - set kI and kD to 0 (noting kD will not change)
//        - set maxSpeed (noting the higher maxSpeed is the greater the initial
//        rotation,
//                          but may be more erratic)
//        - set turnThreshold = 2 (something small enough to learn the
//        appropriate threshold)
//        - set turnTolerance = 1 (something reasonable that says we are close
//        enough)
//              (note: turnThreshold > turnTolerance, and they represent
//              boundaries on the error)
//        - set maxIter to something reasonably high (~ something in the 100s)
//    - Next: Learn the correct turnThreshold for Your Robot!!
//        - Note: at the end of the while loop, make sure the following is all
//        uncommented
//            - final error and derivative calculations
//            - print statements for iter, error, and derivative
//        - !! Download and Run !!
//            - We expect iter == maxIter (this means that the code terminated
//            without
//                meeting the expected turnTolerance.)
//        - Round up the error printed to the screen and set turnThreshold to
//        that value.
//        - Now start to slowly turn on kI <= kP
//        - !! Download and Run !!
//    - Finally: Tune kI
//        - If iter == maxIter
//            - increase kI
//        - If error < 0
//            - decrease kI
//********//Have fun tuning and learning how these parameters work
// together//********//

// Used to count the number of turns for output data only
int turnCount = 0;
// Relative degree tracking
int angleTracker = 0;
// Weighted factor of porportion error
double kP = 0.19;
// Weighted factor of integral error
double kI = 0.01;
// Weighted factor of the derivated error
double kD = .1;
// Max speed in Volts for motors
double maxSpeed = 6;
// The angle difference from error when integral adjustments turns on
int turnThreshold = 16;
// Tolerance for approximating the target angle
double turnTolerance = .5;
// Total number of iterations to exit while loop
int maxIter = 300;

// Turning Function
void turnPID(double angleTurn) {
  //  Distance to target in degrees
  double error = 0;
  //  Error degree from the last iteration
  double prevError = 0;
  // Derivative of the error. The slope between iterations
  double derivative = 0;
  // Accumulated error after threashold. Summing error
  double integral = 0;
  // Iterations of the loop. Counter used to exit loop if not converging
  double iter = 0;

  // Used for Relative Coordinates. For absolute coordinates, comment out
  // following lines for angleTracker
  angleTracker += angleTurn;
  angleTurn = angleTracker % 360;

  // Automated error correction loop
  while (fabs(TurnGyroSmart.rotation(degrees) - angleTurn) > turnTolerance && iter < maxIter) {
    iter += 1;
    error = angleTurn - TurnGyroSmart.rotation(degrees);
    derivative = error - prevError;
    prevError = error;

    // Checking if error passes threshold to build the integral
    if (fabs(error) < turnThreshold && error != 0) {
      integral += error;
    } else {
      integral = 0;
    }

    // Voltage to use. PID calculation
    double powerDrive = error * kP + derivative * kD + integral * kI;

    // Capping voltage to max speed
    if (powerDrive > maxSpeed) {
      powerDrive = maxSpeed;
    } else if (powerDrive < -maxSpeed) {
      powerDrive = -maxSpeed;
    }

    // Send to motors
    LeftDriveSmart.spin(forward, powerDrive, voltageUnits::volt);
    RightDriveSmart.spin(forward, -powerDrive, voltageUnits::volt);

    this_thread::sleep_for(15);
  }

  // Angle achieved, brake robot
  LeftDriveSmart.stop(brake);
  RightDriveSmart.stop(brake);

  // Tuning data, output to screen
  turnCount += 1;
  error = angleTurn - TurnGyroSmart.rotation(degrees);
  derivative = error - prevError;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Turn #: %d", turnCount);
  Controller1.Screen.setCursor(1, 13);
  Controller1.Screen.print("iter: %.0f", iter);
  Controller1.Screen.newLine();
  Controller1.Screen.print("error: %.5f", error);
  Controller1.Screen.newLine();
  Controller1.Screen.print("derivative: %.5f", derivative);
  Controller1.Screen.newLine();
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
