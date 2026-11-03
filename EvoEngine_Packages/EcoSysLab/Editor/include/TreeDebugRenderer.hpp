#pragma once

#include "Camera.hpp"
#include "GpuTreeDebugData.hpp"
#include "GpuTreeMeshData.hpp"
#include "Platform.hpp"

namespace eco_sys_lab_package {
class Tree;
class ShootVisualizer;
class RootVisualizer;
class GpuTreePhysics;
struct GpuTreePhysicsState;

class TreeDebugRenderer {
  struct Frame {
    std::shared_ptr<Buffer> nodes, rest, colors, state, readback;
    std::shared_ptr<DescriptorSet> draw_set, deform_set;
    std::shared_ptr<Image> pick_image;
    std::shared_ptr<ImageView> pick_view;
    std::shared_ptr<FrameSubmissionState> submission;
    uint64_t uploaded_generation = 0;
    uint64_t pick_generation = 0;
    uint64_t pick_serial = 0;
  };
  std::vector<Frame> frames_;
  std::shared_ptr<TreeDebugRenderer> shared_renderer_;
  std::shared_ptr<GraphicsPipeline> pipeline_, impostor_pipeline_;
  std::shared_ptr<ComputePipeline> deform_pipeline_;
  std::shared_ptr<DescriptorSetLayout> layout_, deform_layout_;
  BufferUploadArena upload_arena_;
  std::weak_ptr<Tree> cached_tree_;
  std::weak_ptr<GpuTreePhysics> cached_physics_;
  std::weak_ptr<GpuTreePhysicsState> cached_state_;
  uint64_t shoot_revision_ = UINT64_MAX, root_revision_ = UINT64_MAX, generation_ = 1;
  int shoot_checkpoint_ = -1, root_checkpoint_ = -1;
  glm::vec4 visualizer_settings_{};
  bool connected_pose_ = false;
  std::array<float, 18> parameters_{};
  std::vector<GpuTreeNode> nodes_;
  std::vector<glm::vec4> colors_;
  std::optional<glm::vec2> click_;
  std::weak_ptr<Tree> requested_tree_;
  uint64_t request_serial_ = 0;
  uint32_t shoot_count_ = 0;

 public:
  TreeDebugRenderer() = default;
  explicit TreeDebugRenderer(const std::shared_ptr<TreeDebugRenderer>& shared)
      : shared_renderer_(shared),
        pipeline_(shared->pipeline_),
        impostor_pipeline_(shared->impostor_pipeline_),
        deform_pipeline_(shared->deform_pipeline_),
        layout_(shared->layout_),
        deform_layout_(shared->deform_layout_) {
  }
  bool Prepare();
  void RequestPick(const std::shared_ptr<Tree>& tree, glm::vec2 point) {
    requested_tree_ = tree;
    ++request_serial_;
    click_ = point;
  }
  void PollPick(const std::shared_ptr<Tree>& tree, ShootVisualizer& shoot, RootVisualizer& root);
  void Render(const std::shared_ptr<Tree>& tree, ShootVisualizer& shoot, RootVisualizer& root,
              const std::shared_ptr<Camera>& camera, const glm::mat4& projection_view, bool context = false);
};
}  // namespace eco_sys_lab_package
