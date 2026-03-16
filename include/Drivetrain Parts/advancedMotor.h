#ifndef _ADVANCEDMOTOR_H_
#define _ADVANCEDMOTOR_H_

#include "main.h"  // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include <cstdlib>
#include <type_traits>
#include <vector>

template <typename T>
class advancedMotor{
  private:
    pros::v5::MotorGears gearset;

  public:
    bool activated;
    int port;

    advancedMotor(
      int port,
      pros::v5::MotorGears gearset
    ) {
      this->port = port;
      this->gearset = gearset;
      activated = true;
    }

    void setPort(int port) {
      this->port = port;
    }

    void setGearset(pros::v5::MotorGears gearset) {
      this->gearset = gearset;
    }

    void enable() {
      activated = true;
    }

    void disable() {
      activated = false;
    }

    void toggle() {
      activated = !activated;
    }

    bool isEnabled() {
      return activated;
    }

    // Functions that allow running all of those basic motor functions
    template <typename Func, typename... Args>
    auto invoke(Func function, Args&&... args) {
      // determine return type of invoking a member function on pros::Motor
      using ReturnType = std::invoke_result_t<Func, pros::Motor&, Args...>;

      if constexpr (std::is_same_v<ReturnType, void>) {
        // no return value, just call method on each enabled motor
        if (activated) {
          pros::Motor motor(port, gearset);
          if (!motor.is_installed()) {
            activated = false;
          }else {
            (motor.*function)(std::forward<Args>(args)...);
          }
        }
      } else {
        // if there is a return value, we just do a similar thing but record what its returning
        ReturnType possibleReturn = ReturnType{};
        if (activated) {
          pros::Motor motor(port, gearset);
          // Checks if the port exists
          if (!motor.is_installed()) {
            activated = false;
          }else {
            possibleReturn = ((motor.*function)(std::forward<Args>(args)...));
          }
        }


        return possibleReturn;
      }
    }

    #undef do
    #define do(method, ...) invoke(&pros::Motor::method, ##__VA_ARGS__)

}; 

template <typename T>
class advancedMotorGroup{
  private:
    pros::v5::MotorGears gearset;

  public:
    std::vector<bool> activated;
    std::vector<int> ports;

    advancedMotorGroup(
      std::vector<int> ports,
      pros::v5::MotorGears gearset
    ) {
      this->ports = ports;
      this->gearset = gearset;
      this->activated = std::vector<bool>(ports.size(), true);
    }

    void setPorts(std::vector<int> ports) {
      this->ports = ports;
    }

    void setGearset(pros::v5::MotorGears gearset) {
      this->gearset = gearset;
    }

    void enable(int portNumber) {
      for (int i = 0; i < ports.size(); i++) {
        if (ports.at(i) == portNumber) {
          activated.at(i) = true;
          return;
        }
      }
    }

    void disable(int portNumber) {
      for (int i = 0; i < ports.size(); i++) {
        if (ports.at(i) == portNumber) {
          activated.at(i) = false;
          return;
        }
      }
    }

    void toggle(int portNumber) {
      for (int i = 0; i < ports.size(); i++) {
        if (ports.at(i) == portNumber) {
          activated.at(i) = !activated.at(i);
          return;
        }
      }
    }

    bool isEnabled(int portNumber) {
      for (int i = 0; i < ports.size(); i++) {
        if (ports.at(i) == portNumber) {
          return activated.at(i);
        }
      }
    }

    // Functions that allow running all of those basic motor functions
    template <typename Func, typename... Args>
    auto invokeGroup(Func function, Args&&... args) {
      // determine return type of invoking a member function on pros::Motor
      using ReturnType = std::invoke_result_t<Func, pros::Motor&, Args...>;

      if constexpr (std::is_same_v<ReturnType, void>) {
        // no return value, just call method on each enabled motor
        for (int i = 0; i < ports.size(); i++) {
          if (activated.at(i)) {
            pros::Motor motor(ports.at(i), gearset);
            if (!motor.is_installed()) {
              activated.at(i) = false;
            }else {
              (motor.*function)(std::forward<Args>(args)...);
            }
          }
        }
      } else {
        // if there is a return value, we just do a similar thing but record what its returning
        std::vector<ReturnType> possibleReturn;
        for (int i = 0; i < ports.size(); i++) {
          if (activated.at(i)) {
            pros::Motor motor(ports.at(i), gearset);
            // Checks if the port exists
            if (!motor.is_installed()) {
              activated.at(i) = false;
            }else {
              possibleReturn.push_back((motor.*function)(std::forward<Args>(args)...));
            }
          }
        }

        if (possibleReturn.empty())
          return ReturnType{};

        ReturnType average = ReturnType{};
        if constexpr (std::is_arithmetic_v<ReturnType>) {
          for (const auto& val : possibleReturn) {
            average += val;
          }
          average /= possibleReturn.size();
        }
        return average;
      }
    }

    // Functions that allow running all of those basic motor functions
    template <typename Func, typename... Args>
    auto invokeGroup(std::vector<int> WantedPorts, Func function, Args&&... args) {
      // determine return type of invoking a member function on pros::Motor
      using ReturnType = std::invoke_result_t<Func, pros::Motor&, Args...>;

      if constexpr (std::is_same_v<ReturnType, void>) {
        // no return value, just call method on each enabled motor
        for (int i = 0; i < WantedPorts.size(); i++) {
          if (activated.at(i)) {
            pros::Motor motor(WantedPorts.at(i), gearset);
            if (!motor.is_installed()) {
              activated.at(i) = false;
            }else {
              (motor.*function)(std::forward<Args>(args)...);
            }
          }
        }
      } else {
        // if there is a return value, we just do a similar thing but record what its returning
        std::vector<ReturnType> possibleReturn;
        for (int i = 0; i < WantedPorts.size(); i++) {
          if (activated.at(i)) {
            pros::Motor motor(WantedPorts.at(i), gearset);
            // Checks if the port exists
            if (!motor.is_installed()) {
              activated.at(i) = false;
            }else {
              possibleReturn.push_back((motor.*function)(std::forward<Args>(args)...));
            }
          }
        }

        if (possibleReturn.empty())
          return ReturnType{};

        ReturnType average = ReturnType{};
        if constexpr (std::is_arithmetic_v<ReturnType>) {
          for (const auto& val : possibleReturn) {
            average += val;
          }
          average /= possibleReturn.size();
        }
        return average;
      }
    }

    #undef DO
    #define DO(method, ...) invokeGroup(&pros::Motor::method, ##__VA_ARGS__)

};

#endif //_ADVANCEDMOTOR_H_