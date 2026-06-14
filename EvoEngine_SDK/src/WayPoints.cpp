#include "WayPoints.hpp"
using namespace evo_engine;

void WayPoints::OnCreate() {
}

void WayPoints::OnDestroy() {
  entities.clear();
  speed = 1.0f;
}

void WayPoints::Update() {
}
