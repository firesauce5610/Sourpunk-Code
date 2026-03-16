/*

Name: allControl.cpp

Author: Caden Gross

Description: This file contains any functions that are always running no matter what condition the bot is in.

*/

#include "main.h" // IWYU pragma: keep
#include "drivetrain.h" 
#include "vars.h"
#include "converters.h"  // IWYU pragma: keep
#include "CadensLVGL.h"
#include <cstdio>

void contrlManager() {
    // print position to contrl screen
    pros::Task contrl_task([&]() {
        int contrlScreen = 2; // variable to track which screen to display

        contrl.print(0, 0, "Welcome to the");
        pros::delay(50);
        contrl.print(1, 0, "Smart controller");
        pros::delay(50);
        contrl.print(2, 0, "Display!");
        pros::delay(800);
        contrl.clear();
        pros::delay(50);

        while (true) {
            try {
                // switch screens when up or down is pressed
                if (contrl.get_digital_new_press(DIGITAL_UP)) {
                    contrlScreen++;
                    contrl.clear();
                    pros::delay(50);
                    if (contrlScreen > 5) {
                        contrlScreen = 1;
                    }
                } else if (contrl.get_digital_new_press(DIGITAL_DOWN)) {
                    contrlScreen--;
                    contrl.clear();
                    pros::delay(50);
                    if (contrlScreen < 1) {
                        contrlScreen = 5;
                    }
                }

                // display different information based on the current screen
                switch (contrlScreen) {
                    case 1:
                        // print robot autonomous 
                        contrl.clear();
                        pros::delay(50);
                        contrl.print(0, 0, "X: %f", odom.odomPose.x); // x
                        pros::delay(50);
                        contrl.print(1, 0, "Y: %f", odom.odomPose.y); // x
                        pros::delay(50);
                        contrl.print(2, 0, "Yaw: %f", odom.odomPose.theta); // x
                        pros::delay(50);
                        break;
                    case 2:
                        // print robot autonomous 
                        contrl.print(0, 0, "%s %s", debugScreen.at(0), "                       "); // 1
                        pros::delay(50);
                        contrl.print(1, 0, "%s %s", debugScreen.at(1), "                       "); // 2
                        pros::delay(50);
                        contrl.print(2, 0, "%s %s", debugScreen.at(2), "                       "); // 3
                        pros::delay(50);
                        break;
                    case 3:
                        // print robot autonomous 
                        contrl.clear();
                        pros::delay(50);
                        contrl.print(0, 0, "R: %f", cDTI(right_motors.DO(get_position, 0))); // x
                        pros::delay(50);
                        contrl.print(1, 0, "L: %f", cDTI(left_motors.DO(get_position, 0)));
                        pros::delay(50);
                        contrl.print(2, 0, "VE: %f", cDTI2(vertical_encoder.get_position())); // x
                        pros::delay(50);
                        break;
                    case 4:
                        // print robot drivetrain temps
                        contrl.print(0, 0, "LT: %d", (int)left_motors.DO(get_temperature, 0)); // x
                        pros::delay(50);
                        contrl.print(0, 8, "RT %d", (int)right_motors.DO(get_temperature, 0)); // x
                        pros::delay(50);
                        contrl.clear_line(1);
                        pros::delay(50);
                        //contrl.print(1, 8, "B: %f", optical.get_hue()); // x
                        pros::delay(50);
                        contrl.print(1, 0, "ErX: %f", errorPos.x); // x
                        pros::delay(50);
                        contrl.print(2, 0, "T: %d", (int)(imu.get_yaw())); // x
                        pros::delay(50);
                        contrl.print(2, 8, "ErA: %f", errorPos.theta); // x
                        pros::delay(50);
                        break;
                    case 5:
                        // Calculate the distance between the bot and 0,0 and then yay
                        Position error = DT.calculateDistanceToPoint({24,0,0});
                        contrl.clear();
                        pros::delay(50);
                        contrl.print(0, 0, "STR: %f", error.x); // x
                        pros::delay(50);
                        contrl.print(1, 0, "DEG: %f", error.theta);
                        pros::delay(50);
                        contrl.print(2, 0, "HE: %d", odom.odomPose.theta); // x
                        pros::delay(50);

                }
            } catch (std::exception& e) {
                contrl.print(3, 0, "Error: %s", e.what());
            }
        }
    });
}


void allControlManager() {
    //Run everything specialized (with a bunch of try catch statements obviously lol)
    tryCatch([&]() {terminal.init();}, "Terminal Init Failed (wait how you seeing this)");

    tryCatch([&]() {lv_screen_load(terminal.terminalScreen);}, "Brain Screen Switch Failure");    

    tryCatch([&]() {odom.init();}, "Odom Init Failed");
    
    tryCatch(contrlManager, "Controller Manager Failed");

    terminal.print("Sourpunk 84260B");
}