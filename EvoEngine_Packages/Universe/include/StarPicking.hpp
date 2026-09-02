#pragma once
#include "ComputePipeline.hpp"
#include "StarCluster.hpp"

namespace evo_engine {
class Camera;
}

namespace universe_package {
using namespace evo_engine;
struct StarClusterBatch;

struct alignas(16) StarPickResult {
  glm::dvec4 position_radius{};
  double distance = 0;
  uint32_t index = UINT32_MAX;
  uint32_t valid = 0;
};
struct StarPickPushConstant {
  int32_t camera_index = 0;
  uint32_t count = 0;
  glm::vec2 cursor_uv{};
  glm::vec2 display_size{1};
  float minimum_radius = 3;
  uint32_t ray_valid = 0;
  uint32_t final_pass = 0;
  uint32_t padding[3]{};
};
static_assert(sizeof(StarPickResult) == 48);
static_assert(sizeof(StarPickPushConstant) == 48);
static_assert(offsetof(StarPickPushConstant, final_pass) == 32);

struct StarPickRange {
  uint64_t identity = 0, seed = 0;
  uint32_t offset = 0, count = 0;
  std::weak_ptr<StarCluster> cluster;
};
struct StarPickRequest {
  std::weak_ptr<Camera> camera;
  uint64_t camera_handle = 0, frame = 0, population_revision = 0, generation = 0, click = 0;
  uint64_t reference_generation = 0;
  glm::vec2 cursor_uv{}, display_size{1}, image_origin{};
  float minimum_radius = 3;
  bool valid = false;
  std::vector<StarPickRange> ranges;
};
struct StarPickSnapshot {
  StarPickResult result{};
  uint64_t identity = 0, seed = 0, frame = 0;
  uint32_t ordinal = 0;
  std::weak_ptr<StarCluster> cluster;
};

// CPU publication rules are independent of GPU execution for lifecycle testing.
struct StarPickState {
  StarPickRequest current;
  StarPickSnapshot hovered, selected;
  uint64_t generation = 1, latest_click = 0;
  bool interaction_locked = false;
  void SetInteractionLocked(bool locked);
  void Update(StarPickRequest request, bool clicked);
  void Complete(const StarPickRequest& request, const StarPickResult& result);
};

class StarPicker {
 public:
  StarPickState state;
  std::string status = "Not initialized";
  bool Initialize(const std::shared_ptr<DescriptorSetLayout>& camera_layout,
                  const std::filesystem::path& shader_path = "./UniverseResources/Shaders/Compute/StarPick.slang");
  void EnsureResources(size_t frame_count, size_t capacity);
  void Reset();
  void Consume(uint32_t slot);
  void Record(VkCommandBuffer command, uint32_t slot, const std::shared_ptr<Buffer>& stars,
              const std::shared_ptr<Camera>& camera, int32_t camera_index,
              const std::shared_ptr<DescriptorSet>& camera_descriptor, StarPickRequest request);
  bool Pending() const;
  bool PendingClick() const;
  bool Ready() const {
    return intersection_ && intersection_->Initialized() && reduction_ && reduction_->Initialized();
  }

 private:
  struct Slot {
    std::shared_ptr<Buffer> scratch[2], final_result, staging;
    std::shared_ptr<DescriptorSet> descriptors[2];
    std::shared_ptr<ImageView> depth_view;
    StarPickRequest request;
    bool pending = false;
  };
  std::shared_ptr<DescriptorSetLayout> layout_;
  std::shared_ptr<ComputePipeline> intersection_, reduction_;
  std::vector<Slot> slots_;
  size_t capacity_ = 0;
};
}  // namespace universe_package
