#ifndef _DRIVETRAIN_H_
#define _DRIVETRAIN_H_

#include "main.h"  // IWYU pragma: keep
#include "Drivetrain Parts/odometry.h" // IWYU pragma: keep
#include "Drivetrain Parts/pid.h" // IWYU pragma: keep
#include "Drivetrain Parts/advancedMotor.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "vars.h"
#include <cmath>
#include <iterator>

class Drivetrain{

  private:
    advancedMotorGroup<pros::Motor>& leftSide;
    advancedMotorGroup<pros::Motor>& rightSide;
    PID *lPID; // Lateral PID
    PID *aPID; // Angular PID


    double capSpeed(double speed, double speedCap) {
      if(speed > speedCap) speed = speedCap;
      else if (speed < -speedCap) speed = -speedCap;
      return speed;
    }

    const double MAXIMUM_MOTOR_POWER = 127;

    // Private function for calculating any PID 
    double calculatePID(double error, PID* pid, double speedCap = 1000) {

      // We first find delta time, so that inconsistent running of the function doesn't effect integral or derivative
      const double now = pros::millis();

      double dt = (pid->prevTime == 0) ? 0 : now - pid->prevTime;
      pid->prevTime = now;

      // Now we calcalate derivative (change in error / time passed)
      float derivative = (dt != 0 ) ? (error - pid->prevError) / to_sec(dt) : 0;
      pid->prevError = error;

      // Now the integral (change in error * time passed) 
      pid->integral += error * to_sec(dt);

      // Main PID equation
      double speed = pid->kP * error + pid->kI * pid->integral + pid->kD * derivative;

      return capSpeed(speed, speedCap);
    }

  public:
    // Public values
    bool moving = false;

    // Allows odom to be fully accessed from the drivetrain
    odometry &odom;

        float addedStraightSpeed = 0;
    float addedTurnSpeed = 0;


    //Constructor
    Drivetrain(
      advancedMotorGroup<pros::Motor>& LS,
      advancedMotorGroup<pros::Motor>& RS,
      PID* LateralPID,
      PID* AngularPID,
      odometry& Odometry
    ):  
        leftSide(LS),
        rightSide(RS),
        odom(Odometry),
        lPID(LateralPID),
        aPID(AngularPID)
    {}

    Drivetrain(
      advancedMotorGroup<pros::Motor>& LS,
      advancedMotorGroup<pros::Motor>& RS,
      odometry& Odometry
    ):
        leftSide(LS),
        rightSide(RS),
        odom(Odometry)
    {}


    void turn(float voltage) {
      leftSide.DO(move, voltage);
      rightSide.DO(move, -voltage);
    }

    void drive(float voltage) {
      leftSide.DO(move, voltage);
      rightSide.DO(move, voltage);
    }

    void setAngularPID(PID AngularPID) {
      aPID = new PID(AngularPID.kP, AngularPID.kI, AngularPID.kD);
    }

    void setLateralPID(PID LateralPID) {
      lPID = new PID(LateralPID.kP, LateralPID.kI, LateralPID.kD);
    }

    void driveArcade(double straight, double turn) {
      straight += addedStraightSpeed;
      turn += addedTurnSpeed;
      double left = straight + turn;
      double right = straight - turn;

      // Normaliziation
      double max = fmax(std::abs(left), std::abs(right));
      if (max > MAXIMUM_MOTOR_POWER) {
        left *= MAXIMUM_MOTOR_POWER / max;
        right *= MAXIMUM_MOTOR_POWER / max;
      }

      leftSide.DO(move, left);
      rightSide.DO(move, right);
    }

    void driveFor(float distance, int timeout = 0, int speedCap = 0)
    {

      // Initial check to make sure no other movement is happening
      if (moving) return;

      moving = true;
      bool finishTimer = false;

      // if timeout is 0 we just end when movement is complete
      if (timeout != 0) pros::Task timeoutWait([&]() {
        pros::delay(timeout);
        moving = false;
      });


      while(moving) {
        // Calculate lateral (distance) error
        double lateralError = distance - odom.getFwd();

        // Calculate lateral speed 
        double driveSpeed = calculatePID(lateralError, lPID, speedCap != 0 ? speedCap : lPID->speedCap);

        // Move the robot
        driveArcade(driveSpeed, 0);
        
        // Set Debug PID screen
        char buffer[64];
        sprintf(buffer, "Error: %f", lateralError);
        debugScreen.at(0) = buffer;
        sprintf(buffer, "FWD: %f", odom.getFwd());
        debugScreen.at(1) = buffer;
        sprintf(buffer, "Power: %f", driveSpeed);
        debugScreen.at(2) = buffer;

        // Check for finished movement
        if (std::abs(lateralError) < lPID->min_error && !finishTimer) {
          finishTimer = true;
          pros::Task finalWait([&]() {
            pros::delay(lPID->min_timeout);
            moving = false;
          });
        }

        // Delay
        pros::delay(20);
      }

      // Stop moving
      drive(0);
    }
  
    void turnFor(float degrees, int timeout = 0, int speedCap = 0)
    {

      // Initial check to make sure no other movement is happening
      if (moving) return;

      moving = true;
      bool finishTimer = false;

      // if timeout is 0 we just end when movement is complete
      if (timeout != 0) pros::Task timeoutWait([&]() {
        pros::delay(timeout);
        moving = false;
      });


      while(moving) {

        // Calculate angular (rotation) error
        double angularError = degrees - odom.imu->get_rotation();

        // Calculate angular speed 
        double turnSpeed = calculatePID(angularError, aPID, speedCap != 0 ? speedCap : aPID->speedCap);

        // Move the robot
        driveArcade(0, turnSpeed);

        // Set Debug PID screen
        char buffer[64];
        sprintf(buffer, "Error: %f", angularError);
        debugScreen.at(0) = buffer;
        sprintf(buffer, "Theta: %f", odom.imu->get_rotation());
        debugScreen.at(1) = buffer;
        sprintf(buffer, "Power: %f", turnSpeed);
        debugScreen.at(2) = buffer;

        // Check for finished movement
        if (std::abs(angularError) < aPID->min_error && !finishTimer) {
          finishTimer = true;
          pros::Task finalWait([&]() {
            pros::delay(aPID->min_timeout);
            moving = false;
          });
        }

        // Delay
        pros::delay(10);
      }

      // Stop moving
      drive(0);
    }

    // Unified movement function combining distance and rotation
    void move(float distance, float degrees, int timeout = 0)
    {

      // Initial check to make sure no other movement is happening
      if (moving) return;

      moving = true;
      bool finishTimer = false;
      bool timerDone = false;

      // if timeout is 0 we just end when movement is complete
      if (timeout != 0) pros::Task timeoutWait([&]() {
        pros::delay(timeout);
        moving = false;
      });

      while(moving) {

        /*
        * Calcalate Errors
        */
        // Calculate lateral (distance) error
        double lateralError = distance - odom.getFwd();
        // Calculate angular (rotation) error
        double angularError = degrees - odom.theta;

        // Account for Level 4 (not immediately moving if we need to turn too much)
        lateralError *= std::max(0.0, std::cos(cDTR(angularError)));


        // Calculate lateral speed 
        double driveSpeed = calculatePID(lateralError, lPID, lPID->speedCap);

        // Calculate angular speed 
        double turnSpeed = calculatePID(angularError, aPID, aPID->speedCap);

        // Move the robot
        driveArcade(driveSpeed, turnSpeed);

        // Set this PID's debug screen
        char buffer[64];
        sprintf(buffer, "LatErr: %f", lateralError);
        debugScreen.at(0) = buffer;
        sprintf(buffer, "AngErr: %f", angularError);
        debugScreen.at(1) = buffer;
        sprintf(buffer, "D: %d %s %d", (int)driveSpeed, "  T: ", (int)turnSpeed);
        debugScreen.at(2) = buffer;
        errorPos.x = lateralError;
        errorPos.y = angularError;

        // Check for finished movement
        if ((std::abs(lateralError) < lPID->min_error) && (std::abs(angularError) < aPID->min_error) && !finishTimer) {
          finishTimer = true;

          pros::Task finalWait([&]() {
            if (!timerDone) pros::delay(lPID->min_timeout);
            timerDone = true;
            if (timeout != 0 && (std::abs(lateralError) < lPID->min_error)) {
              moving = false;
            }
            else {
              finishTimer = false;
            }
          });
        }

        // Delay
        pros::delay(20);
      }

      // Stop moving
      drive(0);
    }

    Position calculateDistanceToPoint(Position targetPoint, bool forwards = true, Position *startPoint = nullptr) {
      float distance, degrees = 0;

      float xDiff;
      float yDiff;
      if (startPoint == nullptr) {
        xDiff = odom.x - targetPoint.x;
        yDiff = odom.y - targetPoint.y;
      }else {
        xDiff = startPoint->x - targetPoint.x;
        yDiff = startPoint->y - targetPoint.y;
      }

      // Pretending that the X cord is the opposite and the Y cord diff is the adajcent, we create a kinda right triangle.
      // So what we need is the angle and the hypotenuse of a right triangle
      // First we can of course do simple pythagorean theorem
      double triHypotenuse = sqrt((xDiff * xDiff) + (yDiff * yDiff)); // Sqrt of a*a and b*b, same as the equation

      // Now we need to get the angle, and we will use sin to get it
      double triAngle = atan2(yDiff,xDiff) * (180.0 / M_PI); 

      // Set the final distance and degrees necessary to move
      degrees = (triAngle - odom.theta) - 180;

      if (xDiff > 0 && forwards) {
        forwards = false;
      }else if (xDiff < 0 && !forwards) {
        forwards = true;
      }

      if (degrees > 180) {
        degrees -= 360;
      }
      if (degrees < -180) {
        degrees += 360;
      }
      distance = triHypotenuse;

      if (!forwards) {
        distance = -distance;
        degrees += 180.0;
      }

      return {distance, degrees, degrees};
    }

    void moveToPoint(Position targetPoint, int timeout = 0, bool forwards = true)
    {

      // Initial check to make sure no other movement is happening
      if (moving) return;

      //Calcalate distance and degrees required to move to point
      Position movementRequired = calculateDistanceToPoint(targetPoint, forwards);

      // Set the final distance and degrees necessary to move
      float degrees = movementRequired.theta;
      float distance = movementRequired.x;

      moving = true;
      pros::Task* finalWait = nullptr;

      // if timeout is 0 we just end when movement is complete
      if (timeout != 0) pros::Task timeoutWait([&]() {
        pros::delay(timeout);
        moving = false;
      });

      while(moving) {

        Position movementRequired = calculateDistanceToPoint(targetPoint, forwards);
        degrees = movementRequired.theta;
        distance = movementRequired.x;

        if ((distance > 0 && forwards) || (distance < 0 && !forwards)) {
          movementRequired = calculateDistanceToPoint(targetPoint, !forwards);
        }
        degrees = movementRequired.theta;
        distance = movementRequired.x;

        // Calculate lateral (distance) error
        double lateralError = distance;
        // Calculate angular (rotation) error
        double angularError = degrees;

        // If we are only one wheel turn away, we gotta turn off angular error in case the position calculator gets kinda weird
        // Might improve on later though because I hate this
        if (lateralError < 0.5) {
          angularError = 0;
        }

        // Account for Level 4 (not immediately moving if we need to turn too much)
        lateralError *= std::max(0.0, std::cos(cDTR(angularError)));

        // Calculate lateral speed 
        double driveSpeed = calculatePID(lateralError, lPID, lPID->speedCap);

        // Calculate angular speed 
        double turnSpeed = calculatePID(angularError, aPID, aPID->speedCap);

        // Move the robot
        driveArcade(driveSpeed, turnSpeed);

        // Set this PID's debug screen
        char buffer[64];
        sprintf(buffer, "LatErr: %f", lateralError);
        debugScreen.at(0) = buffer;
        sprintf(buffer, "AngErr: %f", angularError);
        debugScreen.at(1) = buffer;
        sprintf(buffer, "D: %d %s %d", (int)driveSpeed, "  T: ", (int)turnSpeed);
        debugScreen.at(2) = buffer;

        // Check for finished movement
        if (std::abs(lateralError) < lPID->min_error && finalWait == nullptr) pros::Task finalWait([&]() {
            pros::delay(lPID->min_timeout);
            moving = false;
        });               

        // Delay
        pros::delay(20);
      }

      // Stop moving
      drive(0);
    }
};

// Global Declarations
// left motor group
// right motor group
inline advancedMotorGroup<pros::Motor> left_motors({-11, -12, -13}, pros::MotorGears::blue);
inline advancedMotorGroup<pros::Motor> right_motors({14, 15, 16}, pros::MotorGears::blue);

// imu
inline pros::Imu imu(1);
// horizontal and vertical tracking wheel encoder
inline pros::Rotation horizontal_encoder(2);
inline pros::Rotation vertical_encoder(3);

inline odometry odom(
  left_motors,
  right_motors,
  &imu,
  &horizontal_encoder
);

inline PID LateralPID(
  5.77, // The Proportional
  0.678, // The Integral
  0.55, // The Derivative
  0.4, // When the movement is described as over (inches)
  300, // How long until the movement is considered over (in milliseconds)
  47.0127326151 // Converts encoder degrees to inches
);

inline PID LateralPID_default(
  5.77, // The Proportional
  0.678, // The Integral
  0.55, // The Derivative
  0.4, // When the movement is described as over (inches)
  300, // How long until the movement is considered over (in milliseconds)
  47.0127326151 // Converts encoder degrees to inches
);

// 10 Works Best
inline PID AngularPID(
  2.05, // The Proportional
  0.01, // The Integral
  0.13, // The Derivative
  1, // When the movement is described as over
  300, // How long until the movement is considered over (in milliseconds)
  1 // Converts whatever the PID thing is into inches (1 for no conversion)
);

inline PID AngularPID_5(
  8.14, // The Proportional
  0.8, // The Integral
  0.017, // The Derivative
  1, // When the movement is described as over
  300, // How long until the movement is considered over (in milliseconds)
  1 // Converts whatever the PID thing is into inches (1 for no conversion)
);


inline PID AngularPID_10(
  4.58, // The Proportional
  0.16, // The Integral
  0.08, // The Derivative
  1, // When the movement is described as over
  300, // How long until the movement is considered over (in milliseconds)
  1 // Converts whatever the PID thing is into inches (1 for no conversion)
);

inline PID AngularPID_90(
  2.05, // The Proportional
  0.01, // The Integral
  0.13, // The Derivative
  1, // When the movement is described as over
  200, // How long until the movement is considered over (in milliseconds)
  1 // Converts whatever the PID thing is into inches (1 for no conversion)
);

inline PID AngularPID_120(
  1.803, // The Proportional
  0.01, // The Integral
  0.124, // The Derivative
  1, // When the movement is described as over
  300, // How long until the movement is considered over (in milliseconds)
  1 // Converts whatever the PID thing is into inches (1 for no conversion)
);

inline Drivetrain DT(
  left_motors,
  right_motors,
  &LateralPID,
  &AngularPID,
  odom
);


#endif // DRIVETRAIN_H