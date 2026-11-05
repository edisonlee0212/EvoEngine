#pragma once

#include "ComputePipeline.hpp"
#include "GraphicsPipeline.hpp"

namespace eco_sys_lab_package {
struct WindTrailSettings {
  int budget = 2048;
  int emission_stride = 128;
  float duration = 0.2f;
  float max_length = 2.0f;
  float sample_interval = 0.01f;
  float width = 12.0f;
  float opacity = 0.3f;
};

class WindTrailRenderer {
 public:
  static constexpr uint32_t capacity = 2048;
  static constexpr uint32_t samples_per_trail = 32;
  struct Trail {
    glm::uvec4 state{};  // emission sequence, oldest sample, sample count, tracking
  };
  struct UpdateParameters {
    glm::uvec4 emission{};  // total emitted, emission begin, emission count, stride
    glm::vec4 limits{};     // time, duration, maximum length, sample interval
    glm::uvec4 mode{};      // spawn, budget, particle capacity, unused
  };
  WindTrailSettings settings;
  bool Prepare(const std::shared_ptr<evo_engine::Buffer>& particles, uint32_t particle_capacity);
  void Advance(float dt, uint32_t emitted);
  void Emit(uint32_t begin, uint32_t count);
  uint32_t Draw(VkCommandBuffer command, const glm::ivec4& viewport, uint32_t camera_index);

 private:
  void Dispatch(uint32_t begin, uint32_t count, uint32_t emitted, bool spawn);
  float time_ = 0;
  uint32_t particle_capacity_ = 0;
  std::shared_ptr<evo_engine::Buffer> trails_, samples_;
  std::shared_ptr<evo_engine::DescriptorSetLayout> layout_;
  std::shared_ptr<evo_engine::DescriptorSet> set_;
  std::shared_ptr<evo_engine::ComputePipeline> update_;
  std::shared_ptr<evo_engine::GraphicsPipeline> render_;
};
}  // namespace eco_sys_lab_package
