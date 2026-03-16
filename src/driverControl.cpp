/*

Name: driverControl.cpp

Author: Caden Gross

Description: This file contains the driver control settings and the main control loop for the robot.

*/

#include "Drivetrain Parts/advancedMotor.h"
#include "drivetrain.h" 
#include "intake.h"
#include "main.h"
#include "pros/misc.h"
#include "vars.h"
#include <array>

char letter = 'a'; // What letter is being pressed
std::string currentLine = "";
bool pressed = false; // Decides whether the keyboard has been used or not
stopwatch terminalAnimation;


bool checkLetter(pros::controller_digital_e_t button, char wantedLetter) {
    if (contrl.get_digital_new_press(button)) {
        letter = wantedLetter;
        pressed = true;
        return true;
    }else {
        return false;
    }
}

void checkLetterGroup(const std::array<char, 8> &letterGroup) {
    if (checkLetter(DIGITAL_LEFT, letterGroup[0])) return;
    if (checkLetter(DIGITAL_UP, letterGroup[1])) return;
    if (checkLetter(DIGITAL_DOWN, letterGroup[2])) return;
    if (checkLetter(DIGITAL_RIGHT, letterGroup[3])) return;
    if (checkLetter(DIGITAL_Y, letterGroup[4])) return;
    if (checkLetter(DIGITAL_X, letterGroup[5])) return;
    if (checkLetter(DIGITAL_B, letterGroup[6])) return;
    checkLetter(DIGITAL_A, letterGroup[7]);
}

// Return a pointer to the active letter group based on modifier buttons, or nullptr if none.
const std::array<char, 8> *getActiveLetterGroup() {
    static const std::array<char, 8> digits1 = {{'0','1','2','3','4','5','6','7'}};
    static const std::array<char, 8> digits2 = {{'8','9','*','*','*','*','*','*'}};
    static const std::array<char, 8> A_to_H    = {{'a','b','c','d','e','f','g','h'}};
    static const std::array<char, 8> I_to_P    = {{'i','j','k','l','m','n','o','p'}};
    static const std::array<char, 8> Q_to_X    = {{'q','r','s','t','u','v','w','x'}};
    static const std::array<char, 8> Y_to_MISC = {{'y','z',' ','!','_','$','%','*'}}; // '*' = newline

    if (contrl.get_digital(DIGITAL_L2) && contrl.get_digital(DIGITAL_R2)) return &digits1;
    if (contrl.get_digital(DIGITAL_L1) && contrl.get_digital(DIGITAL_R1)) return &digits2;
    if (contrl.get_digital(DIGITAL_L2)) return &A_to_H;
    if (contrl.get_digital(DIGITAL_L1)) return &I_to_P;
    if (contrl.get_digital(DIGITAL_R2)) return &Q_to_X;
    if (contrl.get_digital(DIGITAL_R1)) return &Y_to_MISC;

    return nullptr;
}

void runKeyboard() {
        // Keyboard: pick the active group and check its letters
        if (const auto *activeGroup = getActiveLetterGroup()) {
            checkLetterGroup(*activeGroup);
        }

        if (pressed) {
            if (letter == '*') {
                terminal.print(currentLine);

                if (currentLine == "clear") {
                    terminal.clear();
                }

                if (currentLine.contains("DO")) {
                    currentLine.erase(0, 2); // Removes DO from line
                    advancedMotor<pros::Motor> testedMotor(std::stoi(currentLine), pros::MotorGears::green);
                    testedMotor.do(move, 127);
                }

                if (currentLine.contains("-DO")) {
                    currentLine.erase(0, 3); // Removes -DO from line
                    advancedMotor<pros::Motor> testedMotor(std::stoi(currentLine), pros::MotorGears::green);
                    testedMotor.do(move, -127);
                }

                if (currentLine.contains("NO")) {
                    currentLine.erase(0, 2); // Removes NO from line
                    advancedMotor<pros::Motor> testedMotor(std::stoi(currentLine), pros::MotorGears::green);
                    testedMotor.do(move, 0);
                }
                
                currentLine = "";
            }else {
                currentLine += letter;
            }
            pressed = false;
        }else {
            std::string printedLine = currentLine + '_';
            if (std::sin(terminalAnimation.milli * 0.005) > 0) std::string printedLine = currentLine + ' ';

            terminal.printLine(terminal.currentLine + 1, printedLine);
        }
}

void opcontrol() {

    terminalAnimation.start();

    while (true) {

        // move the robot
        int straight = contrl.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
	    int turn = contrl.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
        DT.driveArcade(straight, turn); 

       // Intake Controls
        intakeMotors.DO(move, 0);
       if (contrl.get_digital(DIGITAL_L1)) {
           intakeMotors.DO(move, 127);
       }
       if (contrl.get_digital(DIGITAL_L2)) {
           intakeMotors.DO(move, -127);
       }
       if (!contrl.get_digital(DIGITAL_LEFT)) {
           if (contrl.get_digital(DIGITAL_R1)) {
               intakeMotor1.do(move, 127);
           }
           if (contrl.get_digital(DIGITAL_R2)) {
               intakeMotor1.do(move, -127);
           }
       }else {
           if (contrl.get_digital(DIGITAL_R1)) {
               intakeMotor2.do(move, 127);
           }
           if (contrl.get_digital(DIGITAL_R2)) {
               intakeMotor2.do(move, -127);
           }
       }


        // Pnuematic Controls
        // this ones actually right (intaker)
        if (contrl.get_digital_new_press(DIGITAL_A)) {
            intaker.toggle();
        }
        // Chamber
        if (contrl.get_digital_new_press(DIGITAL_B)) {
            chamber.toggle();
        }
        //  Hood
        if (contrl.get_digital_new_press(DIGITAL_Y)) {
            hood.toggle();
        }
        // Descore
        if (contrl.get_digital_new_press(DIGITAL_X)) {
            descore.toggle();
        }

        // Innovate - Turn off one side motor 
        if (contrl.get_digital_new_press(DIGITAL_LEFT)) {
            left_motors.disable(-11);
        }
        if (contrl.get_digital_new_press(DIGITAL_RIGHT)) {
            left_motors.disable(14);
        }

        runKeyboard();
        // delay to save resources
        pros::delay(20);
    }
}