#ifndef _PID_H_
#define _PID_H_

class PID {
  private:

  public:
    float kP;
    float kI;
    float kD;
    float min_error;
    float min_timeout;
    float multiplier = 1;
    float slew_rate = 200;

    // PID Real Time Calculations
    float integral = 0;
    float prevTime = 0;
    float prevError= 0;

    // Auto Tuning Variables
    float bestError = 100000;
    float bestkP = 0;
    float bestkD = 0;

    float speedCap = 1000;

    void record() {
      bestkP = kP;
      bestkD = kD;
    }

    PID(
      float P,
      float I,
      float D,
      float minimum_error = 0,
      float minimum_timeout = 0,
      float converter = 1,
      float slew_rate = 200
    ) {
      this->kP = P;
      this->kI = I;
      this->kD = D;
      this->min_error = minimum_error;
      this->min_timeout = minimum_timeout;
      this->multiplier = converter;
      this->slew_rate = slew_rate;
    }


};

#endif // _PID_H_