#pragma once

#include "mpLib/pose.hpp"
#include <cmath>

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

namespace mpLib {
double angleDifference(double angle1, double angle2, int maxLoops = 1000);

double angle180(double angle);
double calcCurvature(Point2D d, Point2D dd);
} // namespace mpLib
