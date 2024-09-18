#include "CosseratRods.hpp"


using namespace eco_sys_lab_plugin;

void CosseratRods::Update(const Parameters& parameters, std::vector<Strand>& strands,
    std::vector<StrandSegment>& strand_segments) {
  //1. Update velocity.
  Jobs::RunParallelFor(strand_segments.size(), [&](const auto& i) {
    auto& segment = strand_segments[i];
    segment.end_velocity += glm::vec3(0, -9.81f, 0) * parameters.d_t * segment.inv_end_mass;
  });

  //2. Update position.
  Jobs::RunParallelFor(strand_segments.size(), [&](const auto& i) {
    auto& segment = strand_segments[i];
    segment.end_position += segment.end_velocity * parameters.d_t;
  });

  //3. Update angular velocity.
  Jobs::RunParallelFor(strand_segments.size(), [&](const auto& i) {
    auto& segment = strand_segments[i];
  });

  // 4. Update rotation.
  Jobs::RunParallelFor(strand_segments.size(), [&](const auto& i) {
    auto& segment = strand_segments[i];
    auto dq = glm::quat(0.f, segment.angular_velocity * parameters.d_t);
    segment.rotation += dq * segment.rotation * .5f;
    segment.rotation = glm::normalize(segment.rotation);
  });

  for (uint32_t iteration = 0; iteration < parameters.solver_iterations; iteration++) {
    for (uint32_t i = 0; i < strand_segments.size(); i++) {
      if (i % 2 == 0) {
        
      }
    }
  }
}
