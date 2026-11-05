#pragma once
#include "GpuTreeSubtreePhysics.hpp"
#include "TreeClusterHierarchy.hpp"

namespace eco_sys_lab_package {
class GpuTreeClusterPhysics {
 public:
  enum class ExecutionMode { Serial, LevelParallel };
  using Settings = GpuTreeSubtreePhysics::Settings;
  using Motion = GpuTreeSubtreePhysics::Motion;
  bool Initialize(const DynamicTreeSkeleton::GpuRestState& rest, const std::vector<TreeClusterRule>& rules,
                  const std::shared_ptr<GpuTreePhysicsState>& state = {});
  bool InitializeDynamic(const DynamicTreeSkeleton::GpuRestState& rest,
                         const std::shared_ptr<GpuTreePhysicsState>& state = {});
  const std::shared_ptr<Buffer>& GetLodSelectionBuffer() const {
    return lod_buffers_[7];
  }
  const std::shared_ptr<Buffer>& GetLodStateBuffer() const {
    return lod_buffers_[0];
  }
  // Rigid entity transforms only; rebuild from growth for scale/topology changes.
  void Step(const Settings& settings, const GlobalTransform& model = {},
            ExecutionMode execution = ExecutionMode::Serial, const GpuTreePhysics::WindInput& wind = {});
  const std::shared_ptr<DescriptorSetLayout>& GetWindDescriptorSetLayout() const {
    return wind_layout_;
  }
  const std::shared_ptr<GpuTreePhysicsState>& GetTreeState() const {
    return tree_;
  }
  const std::shared_ptr<Buffer>& GetMotionBuffer() const {
    return motion_;
  }
  const TreeClusterSelection& GetSelection() const {
    return selection_;
  }
  const std::vector<TreeSubtreeBody>& GetBodies() const {
    return bodies_;
  }

 private:
  friend class GpuTreeClusterForest;
  struct Body {
    glm::vec4 center_mass{}, inertia0{}, inertia1{}, inertia2{};
    glm::vec4 rest_anchor{}, rest_rotation{}, parent_offset{}, relative_rotation{};
    glm::uvec4 links{};  // Rod range start/count, parent body, fixed.
  };
  struct Parameters {
    glm::vec4 rotation{};
    glm::vec3 translation{};
    uint32_t wind_bucket_count = 0;
    glm::vec3 gravity{};
    uint32_t wind_cell_count = 0;
    glm::vec4 wind_density{}, stiffness_damping{};
    glm::vec2 mechanics{};
    uint32_t level_begin = 0, substep = 0;
    glm::vec3 base_stiffness{};
    float wind_cell_size = 1;
    glm::uvec4 counts{};
  };
  static_assert(sizeof(Body) == 144 && sizeof(Parameters) == 128);
  Parameters MakeParameters(const Settings& settings, const GlobalTransform& model,
                            const GpuTreePhysics::WindInput& wind) const;
  std::array<uint64_t, 18> Addresses() const;
  TreeClusterSelection selection_;
  std::vector<TreeSubtreeBody> bodies_;
  glm::vec3 initial_position_{};
  glm::quat initial_rotation_{1, 0, 0, 0};
  std::shared_ptr<GpuTreePhysicsState> tree_;
  std::shared_ptr<DescriptorSetLayout> layout_, wind_layout_;
  std::shared_ptr<DescriptorSet> empty_wind_set_;
  std::array<std::shared_ptr<Buffer>, 2> empty_wind_buffers_;
  std::shared_ptr<DescriptorSet> set_;
  std::shared_ptr<Buffer> properties_, motion_, rods_, vertices_, registry_, work_, schedule_, contributions_,
      level_ranges_, rod_loads_;
  std::array<std::shared_ptr<Buffer>, 8> lod_buffers_;
  std::vector<glm::uvec2> levels_;
  std::array<std::shared_ptr<ComputePipeline>, 6> pipelines_;
  uint32_t rod_count_ = 0, vertex_count_ = 0;
};
// Independent trees share dispatches while retaining separate simulation and rendering buffers.
class GpuTreeClusterForest {
 public:
  enum class Scheduling { Automatic, Serial, Workgroup };
  struct TreeStep {
    GpuTreeClusterPhysics::Settings settings;
    GlobalTransform model{};
    bool running = true;
    TreeClusterLodSettings lod;
    uint32_t pose_revision = 0;
    float mass_scale = 1;
    float bend_damping_time = 0;
    Scheduling scheduling = Scheduling::Automatic;
  };
  bool Initialize(const std::vector<std::shared_ptr<GpuTreeClusterPhysics>>& trees);
  void Step(const std::vector<TreeStep>& steps, const GpuTreePhysics::WindInput& wind = {});

 private:
  struct LodInput {
    glm::uvec4 control{};
    std::array<glm::vec4, 3> rules{};
    glm::vec4 material{1, 0, 0, 0};
  };
  struct Record {
    std::array<uint64_t, 18> addresses;
    GpuTreeClusterPhysics::Parameters parameters;
    LodInput lod;
  };
  static_assert(sizeof(Record) == 352);
  std::vector<std::shared_ptr<GpuTreeClusterPhysics>> trees_;
  std::vector<Record> records_;
  std::shared_ptr<Buffer> records_buffer_, transition_jobs_, transition_dispatch_;
  BufferUploadArena upload_arena_;
  std::shared_ptr<DescriptorSetLayout> layout_;
  std::shared_ptr<DescriptorSet> set_;
  std::array<std::shared_ptr<ComputePipeline>, 8> pipelines_;
  uint32_t reconstruction_count_ = 0;
};
}  // namespace eco_sys_lab_package
