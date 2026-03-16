#ifndef _INTAKE_H_
#define _INTAKE_H_

#include "Drivetrain Parts/advancedMotor.h"
#include "main.h" // IWYU pragma: keep
#include "pros/motors.hpp"
//#include "pros/adi.hpp"
//#include "pros/optical.hpp"


// Global Declarations
//inline pros::Motor intake_Motor1(19, pros::MotorGears::green);
//inline pros::Motor intake_Motor2(20, pros::MotorGears::blue);
//inline pros::MotorGroup intake_Motors({intake_Motor1.get_port(), intake_Motor2.get_port()}, pros::MotorGears::green);


inline advancedMotor<pros::Motor> intakeMotor1(19, pros::MotorGears::green);
inline advancedMotor<pros::Motor> intakeMotor2(20, pros::MotorGears::green);
inline advancedMotorGroup<pros::Motor> intakeMotors({19, 20}, pros::MotorGears::green);
inline pros::adi::Pneumatics chamber('A',false);
inline pros::adi::Pneumatics intaker('B',false);
inline pros::adi::Pneumatics hood('C',false);
inline pros::adi::Pneumatics descore('D',false);
//inline pros::Optical optical(10);

#endif // INTAKE_H