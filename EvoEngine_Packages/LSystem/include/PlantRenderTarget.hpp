#pragma once

#include "LSystem_PCH.hpp"
#include "OrganInstanceChannel.hpp"
#include "OrganMeshChannel.hpp"
#include "OrganStrandsChannel.hpp"

#include "Scene.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace l_system_package {
using namespace evo_engine;

/**
 * @brief One render-side facade per plant.
 *
 * A PlantRenderTarget owns the channels that publish geometry for a single
 * plant instance. The owning L-system component (typically derived from
 * @c LSystemComponentBase) creates one of these in its OnCreate path and
 * destroys it in its OnDestroy / ClearGeometryEntities path. The target keeps
 * channels keyed by an integer @c channel_id chosen by the using package, so
 * the LSystem core makes no assumption about which organs exist or how many
 * channels a single plant uses.
 *
 * Channel ownership pattern:
 *  - Channels are owned by std::unique_ptr inside the target.
 *  - Their child Entities are parented under @c root_entity_.
 *  - On destruction the channels detach their entities individually, then the
 *    target leaves @c root_entity_ alone (typically the L-system component's
 *    owner entity, whose lifetime is managed by the caller).
 *
 * Threading:
 *  - GetOrCreate* must run on the main thread (Scene mutation).
 *  - The channels' Stage methods are safe to call from worker threads.
 *  - FlushPending must run on the main thread, between simulation step and
 *    the next render submission.
 */
class PlantRenderTarget {
 public:
  PlantRenderTarget(const std::shared_ptr<Scene>& scene, const Entity& root_entity)
      : scene_(scene), root_entity_(root_entity) {
  }

  PlantRenderTarget(const PlantRenderTarget&) = delete;
  PlantRenderTarget& operator=(const PlantRenderTarget&) = delete;
  PlantRenderTarget(PlantRenderTarget&&) = delete;
  PlantRenderTarget& operator=(PlantRenderTarget&&) = delete;

  ~PlantRenderTarget() = default;

  /// Returns the existing mesh channel for @p channel_id, or creates one
  /// parented under the root entity. The channel name is used purely for
  /// diagnostics in the scene hierarchy.
  OrganMeshChannel* GetOrCreateMeshChannel(int channel_id, const std::string& name) {
    if (const auto it = mesh_channels_.find(channel_id); it != mesh_channels_.end()) {
      return it->second.get();
    }
    const auto scene = scene_.lock();
    if (!scene) {
      return nullptr;
    }
    auto channel = std::make_unique<OrganMeshChannel>(scene, root_entity_, name);
    auto* raw = channel.get();
    mesh_channels_.emplace(channel_id, std::move(channel));
    return raw;
  }

  OrganInstanceChannel* GetOrCreateInstanceChannel(int channel_id, const std::string& name,
                                                   const std::shared_ptr<Mesh>& instance_mesh,
                                                   const std::shared_ptr<Material>& instance_material) {
    if (const auto it = instance_channels_.find(channel_id); it != instance_channels_.end()) {
      return it->second.get();
    }
    const auto scene = scene_.lock();
    if (!scene) {
      return nullptr;
    }
    auto channel = std::make_unique<OrganInstanceChannel>(scene, root_entity_, name, instance_mesh, instance_material);
    auto* raw = channel.get();
    instance_channels_.emplace(channel_id, std::move(channel));
    return raw;
  }

  OrganStrandsChannel* GetOrCreateStrandsChannel(int channel_id, const std::string& name) {
    if (const auto it = strands_channels_.find(channel_id); it != strands_channels_.end()) {
      return it->second.get();
    }
    const auto scene = scene_.lock();
    if (!scene) {
      return nullptr;
    }
    auto channel = std::make_unique<OrganStrandsChannel>(scene, root_entity_, name);
    auto* raw = channel.get();
    strands_channels_.emplace(channel_id, std::move(channel));
    return raw;
  }

  /// Drops a single channel (and its child Entity). Useful when an organ
  /// modality is removed mid-simulation.
  void RemoveMeshChannel(int channel_id) {
    mesh_channels_.erase(channel_id);
  }
  void RemoveInstanceChannel(int channel_id) {
    instance_channels_.erase(channel_id);
  }
  void RemoveStrandsChannel(int channel_id) {
    strands_channels_.erase(channel_id);
  }

  /// Drops every channel; called by L-system components on
  /// ClearGeometryEntities.
  void ClearAllChannels() {
    mesh_channels_.clear();
    instance_channels_.clear();
    strands_channels_.clear();
  }

  /// Drains pending payloads from every owned channel. Returns the number of
  /// channels that actually uploaded. Must run on the main thread.
  std::size_t FlushPending() {
    std::size_t flushed = 0;
    for (auto& [id, ch] : mesh_channels_) {
      if (ch->Flush())
        ++flushed;
    }
    for (auto& [id, ch] : instance_channels_) {
      if (ch->Flush())
        ++flushed;
    }
    for (auto& [id, ch] : strands_channels_) {
      if (ch->Flush())
        ++flushed;
    }
    return flushed;
  }

  /// Counts pending publishes across every channel (without draining).
  [[nodiscard]] std::size_t CountPending() const {
    std::size_t pending = 0;
    for (const auto& [id, ch] : mesh_channels_) {
      if (ch->HasPending())
        ++pending;
    }
    for (const auto& [id, ch] : instance_channels_) {
      if (ch->HasPending())
        ++pending;
    }
    for (const auto& [id, ch] : strands_channels_) {
      if (ch->HasPending())
        ++pending;
    }
    return pending;
  }

  [[nodiscard]] Entity GetRootEntity() const noexcept {
    return root_entity_;
  }

  [[nodiscard]] const std::unordered_map<int, std::unique_ptr<OrganMeshChannel>>& GetMeshChannels() const noexcept {
    return mesh_channels_;
  }
  [[nodiscard]] const std::unordered_map<int, std::unique_ptr<OrganInstanceChannel>>& GetInstanceChannels()
      const noexcept {
    return instance_channels_;
  }
  [[nodiscard]] const std::unordered_map<int, std::unique_ptr<OrganStrandsChannel>>& GetStrandsChannels()
      const noexcept {
    return strands_channels_;
  }

 private:
  std::weak_ptr<Scene> scene_;
  Entity root_entity_{};

  std::unordered_map<int, std::unique_ptr<OrganMeshChannel>> mesh_channels_;
  std::unordered_map<int, std::unique_ptr<OrganInstanceChannel>> instance_channels_;
  std::unordered_map<int, std::unique_ptr<OrganStrandsChannel>> strands_channels_;
};

}  // namespace l_system_package
