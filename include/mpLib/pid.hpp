#pragma once

#include <cmath>

#include "mpLib/utils.hpp"

namespace mpLib {

class PID {
private:
  double target = 0;
  double position = 0;

  double power = 0;
  double maxPower = 1;

  double kP;
  double kI;
  double kD;

  double error;
  double totalError = 0;
  double prevError = 0;
  double derivitive;

  double integralBound = 3000;
  double maxIntegral = 30;

  bool turnPid = false;

  double calculatePidValues(double input) {
    position = input;
    this->error = target - position;

    if (turnPid) {
      this->error = angle180(target - position);
    }

    this->derivitive = this->error - this->prevError;

    if (fabs(error) < integralBound) {
      totalError += error;
    } else {
      totalError = 0;
    }

    totalError = fabs(totalError) > maxIntegral ? sgn(totalError) * maxIntegral
                                                : totalError;

    this->power = (error * kP) + (derivitive * kD) + (totalError * kI);

    this->prevError = this->error;

    return this->power;
  }

public:
  PID();
  PID(double kP, double kI, double kD, double target = 0, double position = 0,
      bool turnPid = false);

  double update(double input);

  void operator=(PID pid) {
    this->kP = pid.getKP();
    this->kI = pid.getKI();
    this->kD = pid.getKD();
    this->integralBound = pid.getIntegralBound();
    this->maxIntegral = pid.getMaxIntegral();
  }

  void operator=(PID *pid) {
    this->kP = pid->getKP();
    this->kI = pid->getKI();
    this->kD = pid->getKD();
    this->integralBound = pid->getIntegralBound();
    this->maxIntegral = pid->getMaxIntegral();
  }

  double getDerivitive() { return derivitive; }

  double getError() { return error; }

  double getKP() { return kP; }

  void setKP(double kP) { this->kP = kP; }

  double getKI() { return kI; }

  void setKI(double kI) { this->kI = kI; }

  double getKD() { return kD; }

  void setKD(double kD) { this->kD = kD; }

  double getIntegralBound() { return this->integralBound; }

  void setIntegralBound(double integralBound) {
    this->integralBound = integralBound;
  }

  double getMaxIntegral() { return this->maxIntegral; }

  void setMaxIntegral(double maxIntegral) { this->maxIntegral = maxIntegral; }

  bool getTurnPid() { return turnPid; }

  void setTurnPid(bool turnPid) { this->turnPid = turnPid; }

  void setTarget(double target) { this->target = target; }

  double getTarget() { return target; }

  void setPosition(double target) { this->target = target; }

  double getPosition() { return position; }

  double getPower() { return power; }

  void setMaxPower(double maxPower) { this->maxPower = maxPower; }

  double getMaxPower() { return maxPower; }

  void reset() {
    this->prevError = 0;
    this->error = 0;
    this->derivitive = 0;
  }
};
} // namespace mpLib