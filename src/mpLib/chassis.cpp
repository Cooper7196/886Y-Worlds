#include "mpLib/chassis.hpp"
#include "mpLib/pid.hpp"

namespace mpLib {

void Chassis::initialize() { this->Imu->reset(true); }

void Chassis::initControllers(PID *headingPID, PID *drivePID, double max_acc,
                              double max_dec) {
  this->profileGenerator = new ProfileGenerator();
  this->headingPID = headingPID;
  this->headingPID->setTurnPid(true);
  this->drivePID = drivePID;
  this->constraints =
      new Constraints(this->rpm * this->wheelDiameter * M_PI / 60, max_acc,
                      this->driftFriction, max_dec, 0, this->trackWidth);
}

void Chassis::followPath(virtualPath *path) {
  this->profileGenerator->generateProfile(path, this->constraints);
  auto profile = this->profileGenerator->getProfile();

  std::cout << profile.size() << std::endl;

  PID headingPID(0.05, 0, 0);
  headingPID.setTurnPid(true);
  double distTravelled = 0.1;
  double initialHeading = this->getHeading();

  int lastPos = 0;
  while (true) {
    auto optPoint = this->profileGenerator->getProfilePoint(distTravelled);

    if (optPoint.has_value() == false) {
      break;
    }
    auto point = optPoint.value();
    headingPID.setTarget(point.pose.theta);
    double headingCorrection = headingPID.update(this->getHeading());
    double angularVel = point.omega - headingCorrection;
    double leftVel = point.vel - angularVel * this->trackWidth / 2;
    double rightVel = point.vel + angularVel * this->trackWidth / 2;

    double leftRPM = leftVel * 60 / (M_PI * this->wheelDiameter);
    double rightRPM = rightVel * 60 / (M_PI * this->wheelDiameter);
    leftRPM = leftRPM / this->rpm * 200;
    rightRPM = rightRPM / this->rpm * 200;
    std::cout << point.pose.theta << " " << this->getHeading() << " "
              << headingCorrection << std::endl;
    this->tankVelocity(leftRPM, rightRPM);

    distTravelled += point.vel * 0.01;
    // distTravelled += (leftMotors->get_position() - lastPos) / 360 *
    //                  this->wheelDiameter * M_PI;
    lastPos = leftMotors->get_position();
    pros::delay(10);
  }
  this->tankVoltage(0, 0);
}

void Chassis::followPath(virtualPath *path, double maxVel, double maxAcc,
                         double maxDec) {
  this->constraints->max_acc = maxAcc;
  this->constraints->max_vel = maxVel / 60 * this->wheelDiameter * M_PI;
  this->constraints->max_dec = maxDec;
  this->followPath(path);
}

void Chassis::swingTo(float targetHeading, bool isLeft, int timeout) {
  int time = pros::millis();
  headingPID->setTarget(targetHeading);

  // headingPID->setPosition(this->getHeading());
  double error = 10000;
  while (fabs(error) > 0.5 && pros::millis() - time < timeout) {
    double power = headingPID->update(this->getHeading());
    error = headingPID->getError();
    std::cout << headingPID->getTarget() << " " << power << " " << error << " "
              << this->getHeading() << std::endl;

    if (isLeft) {
      tank(power, 0);
    } else {
      tank(0, power);
    }
    pros::delay(10);
  }
  tankVoltage(0, 0);
  return;
}

void Chassis::driveDistancePID(float distance, float maxSpeed, int timeout) {
  int time = pros::millis();
  this->leftMotors->tare_position();
  drivePID->setTarget(distance);
  // drivePID->setPosition(0);
  headingPID->setTarget(this->getHeading());

  double error = 10000;

  while (fabs(error) > 0.5 && pros::millis() - time < timeout) {
    double position =
        this->leftMotors->get_position() / 360 * this->wheelDiameter * M_PI;
    double power = drivePID->update(position);
    double turnPower = headingPID->update(this->getHeading());
    if (power > maxSpeed) {
      power = maxSpeed;
    }
    if (power < -maxSpeed) {
      power = -maxSpeed;
    }
    error = drivePID->getError();
    tank(power + turnPower, power - turnPower);
    pros::delay(10);
  }
  tankVoltage(0, 0);
  return;
}

void Chassis::tankVoltage(int leftVoltage, int rightVoltage) {
  leftMotors->move_voltage(leftVoltage);
  rightMotors->move_voltage(rightVoltage);
}

void Chassis::tank(int leftVoltage, int rightVoltage) {
  leftMotors->move(leftVoltage);
  rightMotors->move(rightVoltage);
}
void Chassis::tankVelocity(int leftVel, int rightVel) {
  leftMotors->move_velocity(leftVel);
  rightMotors->move_velocity(rightVel);
}
void Chassis::setConstraints(float maxVel, float maxAccel) {
  constraints = new Constraints(maxVel, maxAccel, this->driftFriction, maxAccel,
                                0, this->trackWidth);
}

void Chassis::setConstraints(float maxVel, float maxAccel, float maxDecel) {
  constraints = new Constraints(maxVel, maxAccel, this->driftFriction, maxDecel,
                                0, this->trackWidth);
}
double Chassis::getHeading() { return Imu->get_heading(); }
} // namespace mpLib