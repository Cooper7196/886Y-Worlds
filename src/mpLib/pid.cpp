
#include "mpLib/pid.hpp"

namespace mpLib {

PID::PID() {
  this->kP = 0;
  this->kI = 0;
  this->kD = 0;
  this->target = 0;
  this->position = 0;
}

PID::PID(double kP, double kI, double kD, double target, double position,
         bool turnPid) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->target = target;
  this->position = position;
  this->turnPid = turnPid;
}

double PID::update(double input) { return calculatePidValues(input); }

} // namespace mpLib
