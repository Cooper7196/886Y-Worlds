#pragma once

#include "Eigen/Dense"
#include "mpLib/pose.hpp"
namespace mpLib {

class Point2D {
public:
  Point2D(double x, double y) {
    this->x = x;
    this->y = y;
  }
  Point2D() {
    this->x = 0;
    this->y = 0;
  }
  double x;
  double y;
};
class virtualPath {
public:
  virtual Point2D getPoint(double t) = 0;
  virtual Point2D getDerivative(double t) = 0;
  virtual Point2D getSecondDerivative(double t) = 0;
  virtual double getCurvature(double t) = 0;
  virtual double getCurvature(Point2D d, Point2D dd) = 0;
};

class CubicBezier : public virtualPath {
public:
  CubicBezier(const Point2D &p0, const Point2D &p1, const Point2D &p2,
              const Point2D &p3);

  Point2D getPoint(double t);
  Point2D getDerivative(double t);
  Point2D getSecondDerivative(double t);
  double getCurvature(double t);
  double getCurvature(Point2D d, Point2D dd);

private:
  Point2D p0;
  Point2D p1;
  Point2D p2;
  Point2D p3;
  // 2x4 matrix of control points
  Eigen::Matrix<double, 4, 2> P;

  Eigen::Matrix<double, 4, 4> coefficients;
  Eigen::Matrix<double, 3, 4> derivativeCoef;
  Eigen::Matrix<double, 2, 4> second_derivative_coef;
  double curvature;
};

class Constraints {
public:
  Constraints(double max_vel, double max_acc, double friction_coef,
              double max_dec, double max_jerk, double track_width);
  double maxSpeed(double curvature);
  std::pair<double, double> wheelSpeeds(double angularVel, double vel);
  double max_vel;
  double max_acc;
  double friction_coef;
  double max_dec;
  double max_jerk;
  double track_width;
};

class ProfilePoint {
public:
  ProfilePoint(double x, double y, double theta, double curvature, double t,
               double vel, double accel);
  ProfilePoint(double dist, double vel);
  double x;
  double y;
  double theta;
  double curvature;
  double t;
  double vel;
  double accel;
  double dist;
};

class ChassisSpeeds {
public:
  ChassisSpeeds(double vel, double omega, double accel, Pose pose) {
    this->vel = vel;
    this->omega = omega;
    this->accel = accel;
    this->pose = pose;
  }
  double vel;
  double omega;
  double accel;
  Pose pose;
};

class ProfileGenerator {
public:
  ProfileGenerator();
  void generateProfile(virtualPath *path, Constraints *constraints,
                       double dd = 0.1);
  std::optional<ChassisSpeeds> getProfilePoint(double d);
  auto getProfile() { return profile; }

private:
  Constraints *constraints;
  std::vector<ProfilePoint> profile;
  double dd;
  double duration;
};

class TrapezoidalProfile {
public:
  TrapezoidalProfile(Constraints *constraints, double length,
                     double start_vel = 0, double end_vel = 0);
  double get_vel_at_dist(double dist);

private:
  Constraints *constraints;
  double length;
  double start_vel;
  double end_vel;
  double cruise_vel;
  double accel_dist;
  double decel_dist;
};
} // namespace mpLib