#pragma once
#include "mpLib/PID.hpp"
#include "mpLib/motion_profiling.hpp"
#include "mpLib/pose.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace mpLib {
class Chassis {
public:
  Chassis(const std::initializer_list<int8_t> &leftPorts,
          const std::initializer_list<int8_t> &rightPorts, const int8_t imuPort,
          float trackWidth, float wheelDiameter, float rpm, float driftFriction)
      : leftMotors(std::make_shared<pros::MotorGroup>(leftPorts)),
        rightMotors(std::make_shared<pros::MotorGroup>(rightPorts)),
        Imu(std::make_shared<pros::Imu>(imuPort)), trackWidth(trackWidth),
        wheelDiameter(wheelDiameter), rpm(rpm), driftFriction(driftFriction) {}
  void initialize();
  void initControllers(PID *headingPID, PID *drivePID, double max_acc,
                       double max_dec);

  void tankVoltage(int leftVoltage, int rightVoltage);
  void tank(int leftVoltage, int rightVoltage);
  void tankVelocity(int leftVel, int rightVel);
  void tankVelocityCustom(int leftVel, int rightVel);
  void turnToHeading(float targetHeading, int timeout);
  void driveDistance(float distance);
  void driveDistancePID(float distance, float maxSpeed = 127,
                        int timeout = 100000);
  void driveToPose(Pose targetPose);
  void followPath(mpLib::virtualPath *, bool reversed = false);
  void followPath(virtualPath *path, bool reversed, double maxVel,
                  double maxAcc, double maxDec, double frictionCoef);
  void setConstraints(float maxVel, float maxAccel, float driftFriction);
  void setConstraints(float maxVel, float maxAccel, float maxDecel,
                      float driftFriction);
  void swingTo(float targetHeading, bool isLeft, int timeout = 100000);
  double getHeading();

  void waitUntilSettled();
  void waitUntil(double dist);

private:
  void requestMotionStart();
  void endMotion();
  void followPathInternal(mpLib::virtualPath *, bool reversed = false);
  double calcDistTravelled(double deltaLeft, double leftOffset,
                           double deltaAngle);

  std::shared_ptr<pros::MotorGroup> leftMotors;
  std::shared_ptr<pros::MotorGroup> rightMotors;
  std::shared_ptr<pros::Imu> Imu;

  float trackWidth;
  float wheelDiameter;
  float rpm;
  float driftFriction;

  bool isRunning;
  double curDist;

  double lastLeftVel;
  double lastRightVel;

  ProfileGenerator *profileGenerator;
  Constraints *constraints;
  PID *headingPID;
  PID *drivePID;
};
} // namespace mpLib