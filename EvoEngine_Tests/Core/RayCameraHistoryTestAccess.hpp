#pragma once

#include "Camera.hpp"
#include "RenderGraph.hpp"
#include "RenderLayer.hpp"

namespace evo_engine {
class RayCameraHistoryTestAccess {
 public:
  static RayCameraHistoryResources& Acquire(
      Camera& camera, const RayCameraHistoryTechnique technique, const uint64_t scene_handle, const VkExtent3D extent,
      const std::function<RayCameraHistoryResources(VkExtent3D)>& resource_factory) {
    return camera.AcquireRayCameraHistory(technique, scene_handle, extent, resource_factory);
  }

  static void Invalidate(Camera& camera, const bool invalidate_restir_history = true) {
    camera.InvalidateRayCameraHistory(invalidate_restir_history);
  }

  static void Release(Camera& camera) {
    camera.ReleaseRayCameraHistory();
  }

  static void Register(RenderLayer& render_layer, const uint64_t handle, const std::shared_ptr<Camera>& camera) {
    render_layer.ray_camera_history_cameras_[handle] = camera;
    render_layer.UpdateRayCameraHistoryPeaks();
  }

  static void Prune(RenderLayer& render_layer, const std::shared_ptr<RenderInstanceStorage>& render_instances) {
    render_layer.PruneRayCameraHistories(render_instances);
  }

  static void Clear(RenderLayer& render_layer) {
    render_layer.ClearRayCameraHistories();
  }

  static void SetFrameCount(Camera& camera, const uint32_t frame_count) {
    camera.frame_count_ = frame_count;
  }

  static std::shared_ptr<DescriptorSet> AcquireOutputDescriptor(
      Camera& camera, const uint32_t frame_index, const uint64_t frame_serial,
      const std::function<std::shared_ptr<DescriptorSet>()>& resource_factory) {
    return camera.AcquireRayCameraOutputDescriptor(frame_index, frame_serial, {}, resource_factory);
  }
};
}  // namespace evo_engine
