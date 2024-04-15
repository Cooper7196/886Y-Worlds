#include "mpLib/utils.hpp"

namespace mpLib {

double angleDifference(double angle1, double angle2, int maxLoops) {
  double difference = angle1 - angle2;
  int loopCount = 0;
  while (fabs(difference) > 3.141592653) {
    difference += 3.141592653 * 2.0 * -sgn(difference);
    loopCount++;
  }
  return difference;
}

double angle180(double angle) {
  return angle - (floor((angle + 180) / 360)) * 360;
}

double calcCurvature(Point2D d, Point2D dd) {
  double denominator = d.x * d.x + d.y * d.y;
  denominator *= denominator * denominator;
  denominator = std::sqrt(denominator);
  double k = (d.x * dd.y - d.y * dd.x) / denominator;
  return k;
}
} // namespace mpLib