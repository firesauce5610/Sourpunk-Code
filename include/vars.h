#ifndef _VARS_H_
#define _VARS_H_

#include "CadensLVGL.h"
#include "main.h" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <cmath>
#include <ctime>

// Global Declarations
inline int autonomousMode = 0;
inline bool lilyMode = false;

inline pros::Controller contrl(pros::E_CONTROLLER_MASTER);

inline cterminal terminal;

// Intake Mode
inline int intakeMode = 0;
inline bool oscillating = false;

// Drivetrain Declarations

class Position {
  public:
    float x = 0;
    float y = 0;
    float theta = 0;

    void reset() {
      x = 0;
      y = 0;
      theta = 0;
    }
};

inline Position errorPos;

inline std::vector<std::string> debugScreen = {
  "Nothing Set",
  "Nothing Set",
  "Nothing Set"
};

const double gpsOffsetX = 6 + (1.0/3);
const double gpsOffsetY = 4;

//Useful try catch thing

template<typename Function>
inline void tryCatch(Function attempt, std::string failure) {
    try {
        attempt();
    } catch (std::exception& e) {
        terminal.print(e.what());
    }
}

class stopwatch {
  private:
    pros::Task* timerTask = nullptr;
    double startFrame;
    double totalPausedFrames;
    double pauseFrame;
    bool activated;

    void attemptInit() {
      if (timerTask == nullptr) {
        timerTask = new pros::Task {[=, this] {
          while(true) {
            milli = pros::millis() - startFrame;
            pros::delay(10);
          }
        }};
      }
    }
  public:
    double milli;

    void start() {
      attemptInit();
      startFrame = pros::millis();
    }

    void pause() {
      pauseFrame = milli;
      timerTask->remove();
    }

    void resume() {
      attemptInit();
    }

    void stop() {
      timerTask->remove();
      milli = 0;
      startFrame = 0;
    }
};

#endif // _VARS_H_