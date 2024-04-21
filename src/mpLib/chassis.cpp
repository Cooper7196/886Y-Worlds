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

void Chassis::requestMotionStart() {
  while (this->isRunning) {
    pros::delay(10);
  }
  this->isRunning = true;
}

void Chassis::waitUntilSettled() {
  while (this->isRunning) {
    pros::delay(10);
  }
}

void Chassis::waitUntil(double dist) {
  while (this->isRunning) {
    if (this->curDist >= dist) {
      break;
    }
    pros::delay(10);
  }
}
void Chassis::endMotion() { this->isRunning = false; }

void Chassis::followPathInternal(virtualPath *path, bool reversed) {
  this->profileGenerator->generateProfile(path, this->constraints);
  auto profile = this->profileGenerator->getProfile();

  std::cout << profile.size() << std::endl;

  // PID headingCorrectPID(0, 0, 0);
  PID headingCorrectPID(0.07, 0, 0);
  headingCorrectPID.setTurnPid(true);
  this->curDist = 0.1;
  double initialHeading = this->getHeading();
  double lastPos = 0;
  double lastHeading = initialHeading;

  double timeBasedDist = 0.1;
  double sensorBasedDist = 0.1;
  while (true) {
    auto optPoint = this->profileGenerator->getProfilePoint(this->curDist);

    if (optPoint.has_value() == false) {
      break;
    }

    auto point = optPoint.value();
    if (reversed) {
      point.pose.theta += 180;
      point.vel *= -1;
    }
    headingCorrectPID.setTarget(point.pose.theta);
    double headingCorrection = headingCorrectPID.update(this->getHeading());
    double angularVel = point.omega - headingCorrection;
    // std::cout << point.vel << " " << angularVel << " " << headingCorrection
    //           << std::endl;
    double leftVel = (point.vel - angularVel * this->trackWidth / 2);
    double rightVel = (point.vel + angularVel * this->trackWidth / 2);

    double leftRPM = leftVel * 60 / (M_PI * this->wheelDiameter);
    double rightRPM = rightVel * 60 / (M_PI * this->wheelDiameter);

    std::cout << leftRPM << " " << rightRPM << " " << angularVel << std::endl;

    const float ratio =
        std::max(std::fabs(leftRPM), std::fabs(rightRPM)) / this->rpm;
    if (ratio > 1) {
      leftRPM /= ratio;
      rightRPM /= ratio;
    }

    leftRPM = leftRPM / this->rpm * 200;
    rightRPM = rightRPM / this->rpm * 200;
    // std::cout << point.pose.theta << " " << this->getHeading() << " "
    //           << headingCorrection << " " << leftRPM << " " << rightRPM
    //           << std::endl;
    std::cout << leftRPM << " " << rightRPM << std::endl;
    this->tankVelocity(leftRPM, rightRPM);

    // this->curDist += point.vel * (reversed ? -1 : 1) * 0.01;
    timeBasedDist += point.vel * (reversed ? -1 : 1) * 0.01;
    // sensorBasedDist +=
    //     calcDistTravelled((leftMotors->get_position() / 0.75) - lastPos, 5.5,
    //                       Imu->get_rotation() - lastHeading);

    this->curDist = timeBasedDist;
    lastHeading = Imu->get_rotation();
    lastPos = leftMotors->get_position() / 0.75;
    // std::cout << timeBasedDist << " " << sensorBasedDist << " " << lastPos
    //           << std::endl;
    pros::delay(10);
  }
  this->tankVelocity(0, 0);
  this->endMotion();
}

double Chassis::calcDistTravelled(double deltaLeft, double leftOffset,
                                  double deltaAngle) {
  // Odom using IMU and encoders
  double deltaY = deltaLeft / 360 * this->wheelDiameter * M_PI;
  double deltaX = 0;
  double deltaHeading = deltaAngle * M_PI / 180;

  double localX, localY;

  if (deltaHeading == 0) { // prevent divide by 0
    localX = deltaX;
    localY = deltaY;
  } else {
    localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + 0);
    localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading - leftOffset);
  }

  // save previous pose

  // calculate global x and y
  double globalX, globalY;
  globalX = localY * sin(deltaHeading / 2) - localX * cos(deltaHeading / 2);
  globalY = localY * cos(deltaHeading / 2) + localX * sin(deltaHeading / 2);
  return sqrt(globalX * globalX + globalY * globalY);
}

void Chassis::followPath(virtualPath *path, bool reversed) {
  this->requestMotionStart();
  pros::Task task([=]() { this->followPathInternal(path, reversed); });
}

void Chassis::followPath(virtualPath *path, bool reversed, double maxVel,
                         double maxAcc, double maxDec) {
  this->constraints->max_acc = maxAcc;
  this->constraints->max_vel = maxVel / 60 * this->wheelDiameter * M_PI;
  this->constraints->max_dec = maxDec;
  this->followPath(path, reversed);
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
void Chassis::turnToHeading(float targetHeading, int timeout) {
  int time = pros::millis();
  int settleTime = 0;
  headingPID->setTarget(targetHeading);
  // headingPID->setPosition(0);
  double error = 10000;
  while (pros::millis() - time < timeout && settleTime < 100) {
    if (fabs(error) < 0.5) {
      settleTime += 10;
    } else {
      settleTime = 0;
    }

    double power = headingPID->update(this->getHeading());
    error = headingPID->getError();
    tankVelocity(power, -power);
    pros::delay(10);
  }
  tankVelocity(0, 0);
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
void Chassis::tankVelocityCustom(int leftVel, int rightVel) {
  const double kS = 10;
  const double kV = 1;
  const double kA = 0.2;
  leftMotors->move(
      (leftVel * kV + kS * sgn(leftVel) + kA * (leftVel - lastLeftVel) / 0.01) /
      200 * 127);
  rightMotors->move((rightVel * kV + kS * sgn(rightVel) +
                     kA * (rightVel - lastRightVel) / 0.01) /
                    200 * 127);
  lastLeftVel = leftVel;
  lastRightVel = rightVel;
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