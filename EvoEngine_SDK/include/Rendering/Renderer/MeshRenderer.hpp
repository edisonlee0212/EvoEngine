
#pragma once
#include "IPrivateComponent.hpp"
#include "Mesh.hpp"

namespace evo_engine {

/**
 * @brief A class responsible for rendering meshes with materials in the engine.
 */
class MeshRenderer final : public IPrivateComponent {
  friend class RenderInstanceStorage;

  std::shared_ptr<RangeDescriptor> ray_tracing_meshlet_range_;
  std::shared_ptr<RangeDescriptor> ray_tracing_triangle_range_;
  std::shared_ptr<BottomLevelAccelerationStructure> ray_tracing_blas_;
  std::vector<uint32_t> ray_tracing_packed_source_vertex_indices_;
  std::vector<float> morph_weights_;
  std::vector<float> ray_tracing_morph_weights_;
  std::vector<float> pending_ray_tracing_morph_weights_;
  std::shared_ptr<FrameSubmissionState> pending_ray_tracing_submission_state_;
  Bound ray_tracing_bound_{};
  bool ray_tracing_payload_retry_required_ = false;
  Handle ray_tracing_mesh_handle_ = Handle(0);
  uint32_t ray_tracing_geometry_version_ = 0;
  uint32_t morph_weights_version_ = 0;

 public:
  /**
   * @brief Indicates whether the mesh casts shadows.
   */
  bool cast_shadow = true;

  /**
   * @brief Reference to the mesh asset to be rendered.
   */
  AssetRef mesh;

  /**
   * @brief Reference to the material asset applied to the mesh.
   */
  AssetRef material;

  void SetMorphWeights(const std::vector<float>& weights);

  [[nodiscard]] const std::vector<float>& PeekMorphWeights() const;

  void UpdateRayTracingGeometry();

  /**
   * @brief Called when the MeshRenderer is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Collects references to all assets this MeshRenderer depends on.
   *
   * @param list A vector to which the asset references will be added.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Executes actions required after cloning this component.
   *
   * @param target A shared pointer to the newly cloned component.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;
};

}  // namespace evo_engine
