#pragma once

namespace evo_engine {
class VisibilityTesting {
 public:
  struct VisibilityTestParams {
    uint32_t sample_minimum = 10000;
    uint32_t sample_budget = 1 << 20;
    uint32_t sample_depth = 5;
    bool cull_back_faces = false;
  };
  struct VisibilityTestSample {
    uint32_t triangle_index = 0;
    uint32_t sample_count = 0;
  };

  static void GenerateSamples(const std::shared_ptr<Mesh>& mesh,
                              const VisibilityTestParams& visibility_test_params,
                              std::vector<VisibilityTestSample>& output_samples);
  static void VisibilityTest(const std::shared_ptr<Mesh>& mesh,
                             const std::vector<VisibilityTestSample>& input_samples,
                             const VisibilityTestParams& visibility_test_params, std::vector<float>& visibility_results);

  static void VisibilityTest(const std::shared_ptr<Mesh>& mesh,
                             const VisibilityTestParams& visibility_test_params, std::vector<float>& visibility_results);
};

}  // namespace mesh_tracing