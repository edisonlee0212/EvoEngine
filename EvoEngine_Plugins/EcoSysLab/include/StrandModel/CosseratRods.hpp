#pragma once
#include "StrandGroup.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

class CosseratRods {
 public:
  struct Parameters {
    float d_t;
    uint32_t solver_iterations = 50;
  };
  void Update(const Parameters& parameters, std::vector<Strand>& strands, std::vector<StrandSegment>& strand_segments);
};
}  // namespace eco_sys_lab_plugin