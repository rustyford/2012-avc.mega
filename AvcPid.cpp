#include "AvcPid.h"

  // time is in seconds, speed in meters / second
  float AvcPid::compute (float error, float timeDelta, float speed) {
    float a = speed * sin(alpha);
    float optimumError = speed / 2;
    float minimumKd = optimumError / a;
    float kd max(minimumKd, error / a);
    float derivative = (error - previousError) / timeDelta;
    float result = -kp * error - kd * derivative;
    return result;
  }

