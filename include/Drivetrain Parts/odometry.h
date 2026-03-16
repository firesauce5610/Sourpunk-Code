#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "main.h"  // IWYU pragma: keep
#include "converters.h"  // IWYU pragma: keep
#include "advancedMotor.h"
#include "vars.h"
#include <cmath>

class odometry {
    private:
      advancedMotorGroup<pros::Motor>& leftSide;
      advancedMotorGroup<pros::Motor>& rightSide;


      pros::Rotation* horizontal = nullptr;
      pros::Rotation* vertical = nullptr;
      pros::GPS* gps = nullptr;
      pros::Task* trackingTask = nullptr;

      float const LEFTMOTOROFFSET = 6; // 6 inches off
      float const RIGHTMOTOROFFSET = -6; // 6 inches off
      float const HORIZONTALOFFSET = 4;
      float const VERTICALOFFSET = 4;

      float prevFWD = 0;
      float prevTheta = 0;
      float prevSTR = 0;

      void update() {
        // Current Movement Pos

        // ((position right * distance from left wheels) - (position left * distance from right wheels)) / (distance from left wheels - distance from right wheels) 
        // We eventually should replace this with a vertical tracking wheel also being averaged in but with a greater bias than the wheels
        float fwd = (cDTI(rightSide.DO(get_position, 0))*LEFTMOTOROFFSET - cDTI(leftSide.DO(get_position, 0))*RIGHTMOTOROFFSET)/(LEFTMOTOROFFSET-RIGHTMOTOROFFSET); 

        // Has both the option to use IMU, but can also just try grabbing the values from the drivetrain if not possible in a normal situation
        float theta;
        if (this->imu != nullptr)  theta = cDTR(imu->get_yaw());
        else theta = cDTR(cDTI(rightSide.DO(get_position, 0))-cDTI(leftSide.DO(get_position, 0)) / (LEFTMOTOROFFSET-RIGHTMOTOROFFSET));

        float str;
        if (this->horizontal != nullptr) str = cDTI2(horizontal->get_position()) - HORIZONTALOFFSET * std::sin(theta);
        else {
          str = 0; 
          prevSTR = 0;
        }

        // Delta Versions
        float deltaFWD = fwd - prevFWD;
        float deltaTheta = theta - prevTheta;
        float deltaSTR = str - prevSTR;

        // Set the new previous values
        prevFWD = fwd;
        prevTheta = theta;
        prevSTR = str;

        // Local deltas
        float relDeltaX, relDeltaY;

        if (std::abs(deltaTheta) < 1e-6) { // if VERY close to 0, then we just don't take delta for account
            relDeltaX = deltaFWD;
            relDeltaY = deltaSTR;
        } else {
            // Cancels out the amount we've turned
            float r0 = deltaFWD / deltaTheta; 
            float r1 = deltaSTR / deltaTheta; 

            relDeltaX = r0 * std::sin(deltaTheta) - r1 * (1 - std::cos(deltaTheta)); 
            relDeltaY = r1 * std::sin(deltaTheta) + r0 * (1 - std::cos(deltaTheta)); 
        }

        // Update global pose
        odomPose.x += relDeltaX * std::cos(theta) - relDeltaY * std::sin(theta);
        odomPose.y += relDeltaY * std::cos(theta) + relDeltaX * std::sin(theta);
        odomPose.theta = cRTD(theta);

        // Debug
        localPose.x = relDeltaX;
        localPose.y = relDeltaY;
        localPose.theta = cRTD(deltaTheta);
      }

      // Unified movement function combining distance and rotation but only controller outputting
      void getGPSPose(bool imuThetaMode = false)
      {
        Position newPos;
	      newPos.reset();
	      newPos.x = cMTI(gps->get_position_x());
        newPos.y = cMTI(gps->get_position_y());
	      
        if (imuThetaMode) newPos.theta = imu->get_yaw();
        else gps->get_yaw();

	      // Move position using origin rotation 
	      double offsetX = cos(cDTR(newPos.theta)) * (gpsOffsetX) - sin(cDTR(newPos.theta)) * (gpsOffsetY);
	      double offsetY = sin(cDTR(newPos.theta)) * (gpsOffsetX) + cos(cDTR(newPos.theta)) * (gpsOffsetY);

	      newPos.x -= offsetX;
	      newPos.y -= offsetY;

        gpsPose.x = newPos.x;
        gpsPose.y = newPos.y;
        gpsPose.theta = newPos.theta;
      }
      
    public:
      Position odomPose;
      float x = 0;
      float y = 0;
      float theta = 0;
      Position gpsPose;

            pros::Imu* imu = nullptr;

      // Debug variable for tracking the amount of distance thats been moved
      Position localPose;

      //Constructors
      odometry(
        advancedMotorGroup<pros::Motor>& LS,
        advancedMotorGroup<pros::Motor>& RS,
        pros::Imu* InertialSensor,
        pros::Rotation* horizontal_encoder,
        pros::GPS* GPSSensor
      ):
          leftSide(LS),
          rightSide(RS),
          imu(InertialSensor),
          horizontal(horizontal_encoder),
          gps(GPSSensor)  
      {}

      odometry(
        advancedMotorGroup<pros::Motor>& LS,
        advancedMotorGroup<pros::Motor>& RS,
        pros::Imu* InertialSensor,
        pros::Rotation* horizontal_encoder
      ):
          leftSide(LS),
          rightSide(RS),
          imu(InertialSensor),
          horizontal(horizontal_encoder)
      {}

      odometry(
        advancedMotorGroup<pros::Motor>& LS,
        advancedMotorGroup<pros::Motor>& RS,
        pros::Imu* InertialSensor
      ):
          leftSide(LS),
          rightSide(RS),
          imu(InertialSensor)
      {}

      odometry(
        advancedMotorGroup<pros::Motor>& LS,
        advancedMotorGroup<pros::Motor>& RS
      ):
          leftSide(LS),
          rightSide(RS)
      {}
    

      void init(bool gpsUseIMU = false, Position startPose = {0,0,0}) {
          // If statement makes sure that the init function can be ran twice without making any problems or resetting anything
          if (trackingTask == nullptr) {
            // Resets the internal position of the robot
            odomPose.reset();

            trackingTask = new pros::Task {[=, this] {
                // Sets the robots position
                setPose(startPose);

                while (true) {
                    // The main internal lemlib like odometry
                    update();

                    // The gps odometry as a backup
                    if (this->gps != nullptr) getGPSPose(gpsUseIMU);

                    // This sets the more easy to access variables to the internal odomPose
                    x = odomPose.x;
                    y = odomPose.y;
                    theta = odomPose.theta;

                    // Delay (specifically 11 milliseconds for the best version of the linear algebra equations)
                    pros::delay(11);
                }
            }};
          }    
      }

      void setPose(Position newPose) {
        odomPose = newPose;
        if (this->imu != nullptr) imu->set_yaw(newPose.theta);
      }
      
      Position getPose() {
        return odomPose;
      }

      double getFwd() {
        return (cDTI(rightSide.DO(get_position,0))*LEFTMOTOROFFSET - cDTI(leftSide.DO(get_position,0))*RIGHTMOTOROFFSET)/(LEFTMOTOROFFSET-RIGHTMOTOROFFSET); 
      }

      void reset() {
        rightSide.DO(tare_position, 0);
        leftSide.DO(tare_position, 0);
        imu->set_rotation(0);
      }

      void resetWithoutIMU() {
        rightSide.DO(tare_position, 0);
        leftSide.DO(tare_position, 0);
      }
};

#endif // _ODOMETRY_H_