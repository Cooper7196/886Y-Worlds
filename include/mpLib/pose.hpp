#pragma once
namespace mpLib {
class Pose {
public:
  Pose();
  Pose(double x, double y);
  Pose(double x, double y, double theta);

  double distance(Pose pose);
  double angle(Pose pose);

  double x;
  double y;
  double theta;
};
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
} // namespace mpLib
