/*

Name: autonomous.cpp

Author: Caden Gross

Description: This file contains the driver control settings and the main control loop for the robot.

*/

#include "drivetrain.h" 
#include "intake.h"
#include "main.h"
#include "vars.h"
#include "Drivetrain Parts/advancedMotor.h"
#include <cmath>
#include <cstdlib>

// Autonomous Switcher
void competition_initialize();

void autoTuningLateral() {
	int totalRounds = 0;
	int roundsWithOscillation = 0;

	bool dir = 1; // 1 for forward, 0 for backwards

	while (roundsWithOscillation < 20 || totalRounds < 35) {
		// Init variables
		bool oscillating = false;

		// Check for oscillation
		pros::Task checkForOscillating([&]() {
			pros::delay(100);
			bool overshoot = false;

			while (!oscillating) {

				if (errorPos.x * (dir ? 1 : -1) > 0) {
					overshoot = true;
				}else {
					if (overshoot && errorPos.x * (dir ? 1 : -1) <= 1) {
						oscillating = true;
					}
				}
				pros::delay(20);
			}
		});

		// Move
		if (dir) DT.moveToPoint({15,0,0}, 10000,true);
		else DT.moveToPoint({0, 0, 0}, 10000, false);

		pros::delay(500); // Final wait for oscillating check
		checkForOscillating.remove(); // We use remove since theres no real check to stop checkforOscillating if there truly is no oscillating.

		// Calcalate flowchart variables
		if (oscillating) {
			roundsWithOscillation++;
			LateralPID.kD += 0.01;
		}else {
			LateralPID.record();
			LateralPID.kP += 0.01;
		}

		// Start next round
		dir = !dir;
		totalRounds++;

		while ((right_motors.DO(get_temperature, 0) >= 60 || left_motors.DO(get_temperature, 0) >= 60 )) {
			pros::delay(20);
		}
	}

	LateralPID.kD = LateralPID.bestkD;
	LateralPID.kP = LateralPID.bestkP;
}

/*
Tweaking variables
*/
const double OVERSHOOT_REQUIREMENT = 2; // Overshoot requires 2 degrees
const double OVERSHOOT_PUNISHMENT = 0.2; // Removes this much from kP when overshoots

void autoTuningAngular() {
	int totalRounds = 0;
	int roundsWithOscillation = 0;

	while (roundsWithOscillation < 20 || totalRounds < 35) {
		// Init variables
		bool oscillating = false;

		// Check for oscillation
		pros::Task checkForOscillating([&]() {
			pros::delay(100);
			bool overshoot = false;

			while (!oscillating) {

				if (errorPos.theta > 0) {
					overshoot = true;
				}else {
					if (overshoot && errorPos.theta <= 1) {
						oscillating = true;
					}
				}
				pros::delay(20);
			}
		});

		// Move
		DT.turnFor(90, 10000);

		pros::delay(500); // Final wait for oscillating check
		checkForOscillating.remove(); // We use remove since theres no real check to stop checkforOscillating if there truly is no oscillating.

		// Calcalate flowchart variables
		if (oscillating) {
			roundsWithOscillation++;
			AngularPID.kD += 0.01;
		}else {
			AngularPID.record();
			AngularPID.kP += 0.01;
		}

		// Start next round
		totalRounds++;

		while ((right_motors.DO(get_temperature, 0) >= 60 || left_motors.DO(get_temperature, 0) >= 60 )) {
			pros::delay(20);
		}
	}

	AngularPID.kD = AngularPID.bestkD;
	AngularPID.kP = AngularPID.bestkP;
}

void autonomousLeft() {
	odom.reset();             
	DT.move(30.7, 0, 1500);
	DT.setAngularPID(AngularPID_90); // We do 90 degree turn, so we use 90 degree angular
	AngularPID.min_timeout = 50;
	AngularPID.kP  = 2.06; // Less precision but it gets the job done faster
 	intaker.extend();
	// sets the intake to give a lot of effort in the front but not much at the outtake so that we can only hold 4 balls max (1 being our preload)
	intakeMotor1.do(move, -127);
	intakeMotor2.do(move, -5);
	DT.turnFor(-90, 1000);
	odom.resetWithoutIMU();
	LateralPID.min_error = 1;
	DT.setAngularPID(AngularPID_10); // After the 90 degree turn we now go to more precision and correction
	AngularPID.min_error = 1.2;
	DT.move(11.5, -90, 1300);
	// Now we wait for the goal to finish
	chamber.extend(); // Might as well extend the chamber now for extra safety
	intakeMotors.DO(move, -127);
	pros::delay(300);
	errorPos.x = -14;
	LateralPID.min_timeout = 20;
	DT.move(-14.5, -90, 1300); // Moves halfway

	intakeMotors.DO(move, 0); // We need to first stop the intake motors so that the hood can extend without dumping all the blocks out
	DT.setAngularPID(AngularPID_120); // Sets to 120 so it doesn't try to course correct as hard without not doing it at all
	stopwatch timeoutWatch;
	timeoutWatch.start();
	DT.drive(-50); // IF YOU CHANGE THIS NUMBER TO GO FASTER YOU HAVE TO TWEAK THE TORQUE AMOUNT (probably idk)
	// Move carefully into the goal waiting for enough pressure to be put on the left motors to make sure we actually hit a wall
	while(left_motors.DO(get_torque, 0) < 0.23 || timeoutWatch.milli >= 3000) {
		char buffer[64];
        sprintf(buffer, "Torque: %f", left_motors.DO(get_torque, 0));
        debugScreen.at(0) = buffer;
		sprintf(buffer, "ErrorPosX: %f", errorPos.x);
        debugScreen.at(1) = buffer;

		pros::delay(10);
	}
	pros::delay(200);
	DT.drive(-40);
	timeoutWatch.stop();
	hood.extend();
	intaker.retract();
	intakeMotors.DO(move, -127);
	LateralPID.min_error = 0.5;
	LateralPID.min_timeout = 300;
	pros::delay(1200); // We take 1.2 seconds to allow all blocks to be dumped out
	hood.retract();
	DT.drive(0); 
	intakeMotors.DO(move, 0);
	odom.resetWithoutIMU();

	// Now its time for middle goal
	DT.move(11.6, -90, 1500); // Retreat from side goal
	DT.setAngularPID(AngularPID_120);
	DT.turnFor(135, 1300);
	intakeMotors.DO(move, -127);
	chamber.retract();
	odom.resetWithoutIMU();
	LateralPID.speedCap = 65;
	errorPos.x = 15;
	DT.setAngularPID(AngularPID_10);
	DT.move(26, 135, 2000);

	// Turn 180 Solution
	DT.setAngularPID(AngularPID_120);
	AngularPID.min_error = 5;
	AngularPID.min_timeout = 10;
	DT.turnFor(-45, 1000);
	DT.setAngularPID(AngularPID_5);
	intakeMotor1.do(move, -20);
	intakeMotor2.do(move, 20);
	DT.turnFor(-45, 1000);
	imu.set_rotation(odom.theta);
	DT.setAngularPID(AngularPID_10);
	DT.setLateralPID(LateralPID_default);
	
	intakeMotors.DO(move, 0);
	odom.resetWithoutIMU();
	hood.extend();
	LateralPID.kP = 5.9;
	DT.move(-9, -45, 2000);
	intakeMotors.DO(move, -127);
	pros::delay(400);

}

void autonomousSkills() {
	odom.reset();             
	DT.move(30.7, 0, 1500);
	DT.setAngularPID(AngularPID_90); // We do 90 degree turn, so we use 90 degree angular

 	intaker.extend();
	// sets the intake to give a lot of effort in the front but not much at the outtake so that we can only hold 4 balls max (1 being our preload)
	intakeMotors.DO(move, -127);
	DT.turnFor(-90, 1300);
	odom.resetWithoutIMU();
	DT.setAngularPID(AngularPID_10); // After the 90 degree turn we now go to more precision and correction
	AngularPID.min_error = 1.2;
	DT.move(13, -90, 1500);
	// Now we wait for the goal to finish

	intakeMotors.DO(move, -127);
	pros::delay(700);
	DT.drive(-45);
	while (odom.getFwd() > 10) {
		pros::delay(10);
	}
	DT.move(13, -90, 800);
	pros::delay(200);
	intakeMotor1.do(move, -110);
	intakeMotor2.do(move, -90);
	chamber.extend(); // Might as well extend the chamber now for extra safety
	DT.setLateralPID(LateralPID_default);
	DT.setAngularPID(AngularPID_10);
	DT.move(-14.5, -90, 1400); // Moves halfway

	intakeMotors.DO(move, 0); // We need to first stop the intake motors so that the hood can extend without dumping all the blocks out
	DT.setAngularPID(AngularPID_120); // Sets to 120 so it doesn't try to course correct as hard without not doing it at all
	stopwatch timeoutWatch;
	timeoutWatch.start();
	intaker.retract();
	DT.drive(-50); // IF YOU CHANGE THIS NUMBER TO GO FASTER YOU HAVE TO TWEAK THE TORQUE AMOUNT (probably idk)
	// Move carefully into the goal waiting for enough pressure to be put on the left motors to make sure we actually hit a wall
	while(left_motors.DO(get_torque, 0) < 0.23 || timeoutWatch.milli >= 2500) {
		char buffer[64];
        sprintf(buffer, "Torque: %f", left_motors.DO(get_torque, 0));
        debugScreen.at(0) = buffer;
		sprintf(buffer, "ErrorPosX: %f", errorPos.x);
        debugScreen.at(1) = buffer;

		pros::delay(10);
	}
	pros::delay(400);
	DT.drive(-40);
	timeoutWatch.stop();
	hood.extend();
	intakeMotors.DO(move, -127);
	DT.setLateralPID(LateralPID_default);
	pros::delay(1100); // We take 1.2 seconds to allow all blocks to be dumped out
	intakeMotors.DO(move, 0);
	pros::delay(200);
	intakeMotors.DO(move, -127);
	pros::delay(1000);
	DT.drive(0); 
	odom.resetWithoutIMU();

	// Now its time for middle goal
	DT.move(11.6, -90, 1800); // Retreat from side goal
	hood.retract();
	DT.setAngularPID(AngularPID_120);
	DT.turnFor(135, 1300);
	intakeMotors.DO(move, -127);
	odom.resetWithoutIMU();
	LateralPID.speedCap = 65;
	errorPos.x = 15;
	DT.setAngularPID(AngularPID_10);
	DT.move(25, 135, 2000);



	DT.setLateralPID(LateralPID_default);
	DT.setAngularPID(AngularPID_90);
	LateralPID.speedCap = 100;
	DT.turnFor(90, 1400);
	odom.resetWithoutIMU();
	LateralPID.kP = 6.2;
	DT.setAngularPID(AngularPID_10);
	DT.drive(70);
	while (odom.getFwd() < 20) {
		pros::delay(10);
	}
	DT.drive(0);
	pros::delay(200);
	// Moves across the field
	DT.setLateralPID(LateralPID_default);
	DT.move(30, 90, 2000);
	DT.setAngularPID(AngularPID_90);
	DT.turnFor(45, 1000);
	odom.resetWithoutIMU();
	DT.setAngularPID(AngularPID_10);
	DT.drive(80);
	while (odom.getFwd() < 15) {
		pros::delay(10);
	}
	DT.drive(0);
	pros::delay(300);

	// Now we turn into the new goalo
	DT.move(30, 45, 2000);// half aligner for the second side goal AAGWAFASFFWADFAW IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT
	DT.setAngularPID(AngularPID_90);
	DT.turnFor(90, 1600);

	// In the new goal now
	odom.resetWithoutIMU();
	LateralPID.kP = 8;
	LateralPID.kI = 2;
	DT.move(-11, 90, 800);
	DT.setAngularPID(AngularPID_120); // Sets to 120 so it doesn't try to course correct as hard without not doing it at all
	stopwatch timeoutWatch2;
	timeoutWatch2.start();
	intaker.extend();
	DT.drive(-50); // IF YOU CHANGE THIS NUMBER TO GO FASTER YOU HAVE TO TWEAK THE TORQUE AMOUNT (probably idk)
	// Move carefully into the goal waiting for enough pressure to be put on the left motors to make sure we actually hit a wall
	while(left_motors.DO(get_torque, 0) < 0.23 || timeoutWatch2.milli >= 2000) {
		char buffer[64];
        sprintf(buffer, "Torque: %f", left_motors.DO(get_torque, 0));
        debugScreen.at(0) = buffer;
		sprintf(buffer, "ErrorPosX: %f", errorPos.x);
        debugScreen.at(1) = buffer;

		pros::delay(10);
	}
	pros::delay(400);
	DT.drive(-40);
	timeoutWatch2.stop();
	hood.extend();
	intakeMotors.DO(move, -127);
	DT.setLateralPID(LateralPID_default);
	pros::delay(800); // We take 1.2 seconds to allow all blocks to be dumped out
	intakeMotors.DO(move, 0);
	pros::delay(200);
	intakeMotors.DO(move, -127);
	pros::delay(800);
	DT.drive(0); 
	odom.resetWithoutIMU();
	
	DT.setLateralPID(LateralPID_default);
	hood.retract();
	intakeMotors.DO(move, -127);
	chamber.retract();
	LateralPID.speedCap = 80;
	DT.move(30, 90, 2000);
	pros::delay(200);
	DT.drive(-45);
	while (odom.getFwd() > 28) {
		pros::delay(10);
	}
	LateralPID.kP = 7;
	LateralPID.kI = 5;
	DT.move(30, 90, 800);
	pros::delay(200);
	chamber.extend();
	pros::delay(100);

	intakeMotor1.do(move, -110);
	intakeMotor2.do(move, -90);
	DT.move(9, 90, 1300); // Moves halfway

	intakeMotors.DO(move, 0); // We need to first stop the intake motors so that the hood can extend without dumping all the blocks out
	DT.setAngularPID(AngularPID_120); // Sets to 120 so it doesn't try to course correct as hard without not doing it at all
	stopwatch timeoutWatch4;
	timeoutWatch4.start();
	DT.drive(-50); // IF YOU CHANGE THIS NUMBER TO GO FASTER YOU HAVE TO TWEAK THE TORQUE AMOUNT (probably idk)
	// Move carefully into the goal waiting for enough pressure to be put on the left motors to make sure we actually hit a wall
	while(left_motors.DO(get_torque, 0) < 0.23 || timeoutWatch4.milli >= 2500) {
		char buffer[64];
        sprintf(buffer, "Torque: %f", left_motors.DO(get_torque, 0));
        debugScreen.at(0) = buffer;
		sprintf(buffer, "ErrorPosX: %f", errorPos.x);
        debugScreen.at(1) = buffer;

		pros::delay(10);
	}
	


	pros::delay(400);
	DT.drive(-40);
	timeoutWatch4.stop();
	hood.extend();
	intakeMotors.DO(move, -127);
	DT.setLateralPID(LateralPID_default);
	pros::delay(1100); // We take 1.2 seconds to allow all blocks to be dumped out
	intakeMotors.DO(move, 0);
	pros::delay(200);
	intakeMotors.DO(move, -127);
	pros::delay(1000);
	DT.drive(0); 
	odom.resetWithoutIMU();

	DT.setLateralPID(LateralPID_default);
	DT.setAngularPID(AngularPID_10);	


	// Now its time for parkingggggggggggggggggggggggggggg
	DT.move(11.6, 90, 1800); // Retreat from side goal
	intakeMotors.DO(move, 0);
	DT.setAngularPID(AngularPID_90);
	DT.turnFor(0, 1300);
	LateralPID.kI = 3;
	LateralPID.speedCap = 70;
	DT.move(-45, 0, 3500);
	DT.move(-63, 0, 1600);
	DT.turnFor(90, 1200);

	// In the new goal now
	odom.resetWithoutIMU();
	LateralPID.kP = 9;
	LateralPID.kI = 3;
	DT.move(-8, 90, 800);
	DT.setAngularPID(AngularPID_120); // Sets to 120 so it doesn't try to course correct as hard without not doing it at all
	stopwatch timeoutWatch5;
	timeoutWatch2.start();
	intaker.extend();
	DT.drive(-50); // IF YOU CHANGE THIS NUMBER TO GO FASTER YOU HAVE TO TWEAK THE TORQUE AMOUNT (probably idk)
	// Move carefully into the goal waiting for enough pressure to be put on the left motors to make sure we actually hit a wall
	while(left_motors.DO(get_torque, 0) < 0.23 || timeoutWatch5.milli >= 2000) {
		char buffer[64];
        sprintf(buffer, "Torque: %f", left_motors.DO(get_torque, 0));
        debugScreen.at(0) = buffer;
		sprintf(buffer, "ErrorPosX: %f", errorPos.x);
        debugScreen.at(1) = buffer;

		pros::delay(10);
	}
	pros::delay(400);
	DT.drive(-40);
	timeoutWatch5.stop();
	hood.extend();
	intakeMotors.DO(move, -127);
	DT.setLateralPID(LateralPID_default);
	pros::delay(800); // We take 1.2 seconds to allow all blocks to be dumped out
	intakeMotors.DO(move, 0);
	pros::delay(200);
	intakeMotors.DO(move, -127);
	pros::delay(800);
	DT.drive(0); 
	odom.resetWithoutIMU();
	
	DT.setLateralPID(LateralPID_default);
	hood.retract();
	intakeMotors.DO(move, -127);
	chamber.retract();
	LateralPID.speedCap = 80;
	DT.move(30, 90, 2000);
	pros::delay(200);
	DT.drive(-45);
	while (odom.getFwd() > 28) {
		pros::delay(10);
	}
	LateralPID.kP = 7;
	LateralPID.kI = 5;
	DT.move(30, 90, 800);
	pros::delay(200);
	chamber.extend();
	pros::delay(100);

	intakeMotor1.do(move, -110);
	intakeMotor2.do(move, -90);
	DT.move(9, 90, 1300); // Moves halfway

	intakeMotors.DO(move, 0); // We need to first stop the intake motors so that the hood can extend without dumping all the blocks out
	DT.setAngularPID(AngularPID_120); // Sets to 120 so it doesn't try to course correct as hard without not doing it at all
	stopwatch timeoutWatch3;
	timeoutWatch3.start();
	DT.drive(-50); // IF YOU CHANGE THIS NUMBER TO GO FASTER YOU HAVE TO TWEAK THE TORQUE AMOUNT (probably idk)
	// Move carefully into the goal waiting for enough pressure to be put on the left motors to make sure we actually hit a wall
	while(left_motors.DO(get_torque, 0) < 0.23 || timeoutWatch3.milli >= 2500) {
		char buffer[64];
        sprintf(buffer, "Torque: %f", left_motors.DO(get_torque, 0));
        debugScreen.at(0) = buffer;
		sprintf(buffer, "ErrorPosX: %f", errorPos.x);
        debugScreen.at(1) = buffer;

		pros::delay(10);
	}
	


	pros::delay(400);
	DT.drive(-40);
	timeoutWatch3.stop();
	hood.extend();
	intakeMotors.DO(move, -127);
	DT.setLateralPID(LateralPID_default);
	pros::delay(1100); // We take 1.2 seconds to allow all blocks to be dumped out
	intakeMotors.DO(move, 0);
	pros::delay(200);
	intakeMotors.DO(move, -127);
	pros::delay(1000);
	DT.drive(0); 
	odom.resetWithoutIMU();
}


void autonomousRight() {
	// Move over to first goal
	odom.reset();             
	DT.move(30.7, 0, 1500);
	DT.setAngularPID(AngularPID_90); // We do 90 degree turn, so we use 90 degree angular
	AngularPID.min_timeout = 50;
	AngularPID.kP  = 2.06; // Less precision but it gets the job done faster
 	intaker.extend();
	// sets the intake to give a lot of effort in the front but not much at the outtake so that we can only hold 4 balls max (1 being our preload)
	intakeMotor1.do(move, -127);
	intakeMotor2.do(move, -5);
	DT.turnFor(90, 1000);
	odom.resetWithoutIMU();
	LateralPID.min_error = 1;
	DT.setAngularPID(AngularPID_10); // After the 90 degree turn we now go to more precision and correction
	AngularPID.min_error = 1.2;
	DT.move(11.7, 90, 1300);
	// Now we wait for the goal to finish
	chamber.extend(); // Might as well extend the chamber now for extra safety
	intakeMotors.DO(move, -127);
	pros::delay(300);
	errorPos.x = -14;
	LateralPID.min_timeout = 20;
	DT.move(-14.5, 90, 1300); // Moves halfway

	intakeMotors.DO(move, 0); // We need to first stop the intake motors so that the hood can extend without dumping all the blocks out
	DT.setAngularPID(AngularPID_120); // Sets to 120 so it doesn't try to course correct as hard without not doing it at all
	stopwatch timeoutWatch;
	timeoutWatch.start();
	DT.drive(-50); // IF YOU CHANGE THIS NUMBER TO GO FASTER YOU HAVE TO TWEAK THE TORQUE AMOUNT (probably idk)
	// Move carefully into the goal waiting for enough pressure to be put on the left motors to make sure we actually hit a wall
	while(left_motors.DO(get_torque, 0) < 0.23 || timeoutWatch.milli >= 3000) {
		char buffer[64];
        sprintf(buffer, "Torque: %f", left_motors.DO(get_torque, 0));
        debugScreen.at(0) = buffer;
		sprintf(buffer, "ErrorPosX: %f", errorPos.x);
        debugScreen.at(1) = buffer;

		pros::delay(10);
	}
	pros::delay(200);
	DT.drive(-40);
	timeoutWatch.stop();
	hood.extend();
	intaker.retract();
	intakeMotors.DO(move, -127);
	LateralPID.min_error = 0.5;
	LateralPID.min_timeout = 300;
	pros::delay(1200); // We take 1.2 seconds to allow all blocks to be dumped out
	hood.retract();
	DT.drive(0); 
	intakeMotors.DO(move, 0);
	odom.resetWithoutIMU();

// Now its time for middle goal
	DT.move(6, 90, 1500); // Retreat from side goal
	DT.setAngularPID(AngularPID_90);
	DT.turnFor(45, 1300);
	intakeMotors.DO(move, -127);
	chamber.retract();
	odom.resetWithoutIMU();
	LateralPID.speedCap = 60;
	errorPos.x = 15;
	DT.setAngularPID(AngularPID_10);
	
	

}

void autonomous() {
	autonomousSkills();
}