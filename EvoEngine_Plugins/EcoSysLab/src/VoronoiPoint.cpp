#include "VoronoiPoint.hpp"

using namespace kinDS;
// cross product for 3D points
VoronoiPoint<3> kinDS::operator%(const VoronoiPoint<3>& a, const VoronoiPoint<3>& b) {
  return VoronoiPoint<3>{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double kinDS::operator%(const VoronoiPoint<2>& a, const VoronoiPoint<2>& b) {
  return a[0] * b[1] - a[1] * b[0];
}