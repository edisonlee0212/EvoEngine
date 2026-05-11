#include "Point.hpp"

using namespace kinDS;
// cross product for 3D points
Point<3> kinDS::operator%(const Point<3>& a, const Point<3>& b) {
  return Point<3>{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double kinDS::operator%(const Point<2>& a, const Point<2>& b) {
  return a[0] * b[1] - a[1] * b[0];
}