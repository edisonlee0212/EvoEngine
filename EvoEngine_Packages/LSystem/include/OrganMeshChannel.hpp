#pragma once

#include "LSystem_PCH.hpp"
#include "RenderPublishPolicy.hpp"

#include "AssetManager.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Scene.hpp"

#include <chrono>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace l_system_package {
using namespace evo_engine;

/**
 * @brief Plant-agnostic mesh render channel.
 *
 * Wraps a child Entity carrying a MeshRenderer plus its private Mesh and
 * Material assets, and exposes a Stage / Flush interface. Producer code (e.g.
 * an L-system organ mesher running on a worker thread) calls Stage with a POD
 * vertex/index payload; the owning PlantRenderTarget calls Flush from the main
 * thread, which is the only point at which Mesh::SetVertices is invoked.
 *
 * Design constraints:
 *  - Zero botany. Vertices and indices are passed through unchanged.
 *  - No SDK edits. The channel is layered strictly on top of public APIs.
 *  - No compute shaders. All staging happens in CPU std::vector storage.
 *  - The channel owns exactly one child Entity. Lifetime is tied to the
 *    channel; the destructor deletes the Entity (when the Scene is still
 *    alive). This avoids any global registry.
 *
 * Stability notes:
 *  - The high-water vertex / index counts are tracked for diagnostics; they
 *    are also reported by GetStats so downstream packages can react (e.g.
 *    skip detail levels) when individual organs blow past expectations.
 *  - Identical successive payloads are coalesced via a 64-bit hash, which
 *    avoids the most common source of redundant GeometryStorage churn.
 */
class OrganMeshChannel {
 public:
  /// Aggregate of runtime statistics for diagnostics.
  struct Stats {
    std::size_t high_water_vertex_count = 0;
    std::size_t high_water_index_count = 0;
    std::uint64_t flush_count = 0;
    std::uint64_t skipped_by_dedup = 0;
    std::uint64_t skipped_by_rate_limit = 0;
  };

  /**
   * @brief Constructs the channel: creates a child Entity, attaches a
   *        MeshRenderer with a fresh temporary Mesh and Material, and parents
   *        the Entity beneath @p parent.
   *
   * Must be called from the main thread because Scene mutation is not
   * thread-safe.
   */
  OrganMeshChannel(const std::shared_ptr<Scene>& scene, const Entity& parent, const std::string& name)
      : scene_(scene), name_(name) {
    entity_ = scene->CreateEntity(name);
    scene->SetParent(entity_, parent);
    mesh_ = AssetManager::CreateTemporaryAsset<Mesh>();
    material_ = AssetManager::CreateTemporaryAsset<Material>();
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity_).lock();
    renderer->mesh = mesh_;
    renderer->material = material_;
  }

  OrganMeshChannel(const OrganMeshChannel&) = delete;
  OrganMeshChannel& operator=(const OrganMeshChannel&) = delete;
  OrganMeshChannel(OrganMeshChannel&&) = delete;
  OrganMeshChannel& operator=(OrganMeshChannel&&) = delete;

  ~OrganMeshChannel() {
    if (const auto scene = scene_.lock()) {
      if (scene->IsEntityValid(entity_)) {
        scene->DeleteEntity(entity_);
      }
    }
  }

  /**
   * @brief Records a payload for later publication. Safe to call from any
   *        thread; the most recent staged payload always wins.
   *
   * If @c RenderPublishPolicy::defer_to_main_thread is false, this performs
   * the SDK upload immediately and the caller must be on the main thread.
   */
  void Stage(VertexAttributes attributes, std::vector<Vertex> vertices, std::vector<glm::uvec3> triangles) {
    const std::uint64_t payload_hash = ComputePayloadHash(vertices, triangles);
    {
      std::lock_guard<std::mutex> guard(pending_mutex_);
      pending_.attributes = attributes;
      pending_.vertices = std::move(vertices);
      pending_.triangles = std::move(triangles);
      pending_.hash = payload_hash;
      pending_.present = true;
    }
    if (!policy.defer_to_main_thread) {
      Flush();
    }
  }

  /**
   * @brief Convenience wrapper that always uploads immediately. Must be
   *        called on the main thread.
   */
  void Publish(VertexAttributes attributes, std::vector<Vertex> vertices, std::vector<glm::uvec3> triangles) {
    Stage(std::move(attributes), std::move(vertices), std::move(triangles));
    Flush();
  }

  /**
   * @brief Stages an empty payload, which will cause the next Flush to clear
   *        the on-GPU geometry.
   */
  void Clear() {
    Stage({}, {}, {});
  }

  /**
   * @brief Drains the staged payload to the SDK. Must run on the main thread.
   *        Returns true if Mesh::SetVertices was invoked.
   */
  bool Flush() {
    PendingPayload payload;
    {
      std::lock_guard<std::mutex> guard(pending_mutex_);
      if (!pending_.present) {
        return false;
      }
      payload = std::move(pending_);
      pending_ = PendingPayload{};
    }

    if (policy.deduplicate_identical_payloads && payload.hash == last_published_hash_ &&
        payload.vertices.size() == last_published_vertex_count_ &&
        payload.triangles.size() == last_published_triangle_count_) {
      ++stats_.skipped_by_dedup;
      return false;
    }

    if (policy.min_republish_interval_seconds > 0.0f) {
      const double now = NowSeconds();
      if (now - last_publish_time_seconds_ < static_cast<double>(policy.min_republish_interval_seconds)) {
        // Re-stage so the next Flush attempt will still see the payload.
        std::lock_guard<std::mutex> guard(pending_mutex_);
        if (!pending_.present) {
          pending_ = std::move(payload);
        }
        ++stats_.skipped_by_rate_limit;
        return false;
      }
      last_publish_time_seconds_ = now;
    }

    if (!mesh_) {
      return false;
    }
    mesh_->SetVertices(payload.attributes, payload.vertices, payload.triangles);

    last_published_hash_ = payload.hash;
    last_published_vertex_count_ = payload.vertices.size();
    last_published_triangle_count_ = payload.triangles.size();
    stats_.high_water_vertex_count = std::max(stats_.high_water_vertex_count, payload.vertices.size());
    stats_.high_water_index_count = std::max(stats_.high_water_index_count, payload.triangles.size() * std::size_t{3});
    ++stats_.flush_count;
    return true;
  }

  /// True iff a payload is waiting to be drained.
  [[nodiscard]] bool HasPending() const {
    std::lock_guard<std::mutex> guard(pending_mutex_);
    return pending_.present;
  }

  [[nodiscard]] Entity GetEntity() const noexcept {
    return entity_;
  }
  [[nodiscard]] const std::shared_ptr<Mesh>& GetMesh() const noexcept {
    return mesh_;
  }
  [[nodiscard]] const std::shared_ptr<Material>& GetMaterial() const noexcept {
    return material_;
  }
  [[nodiscard]] const std::string& GetName() const noexcept {
    return name_;
  }
  [[nodiscard]] Stats GetStats() const noexcept {
    return stats_;
  }

  /// Tunable knobs. Mutate freely; values are read on the next Flush.
  RenderPublishPolicy policy{};

 private:
  struct PendingPayload {
    VertexAttributes attributes{};
    std::vector<Vertex> vertices;
    std::vector<glm::uvec3> triangles;
    std::uint64_t hash = 0;
    bool present = false;
  };

  static std::uint64_t ComputePayloadHash(const std::vector<Vertex>& vertices,
                                          const std::vector<glm::uvec3>& triangles) {
    std::uint64_t hash = HashBytes(vertices.data(), vertices.size() * sizeof(Vertex));
    hash ^= HashBytes(triangles.data(), triangles.size() * sizeof(glm::uvec3)) + 0x9e3779b97f4a7c15ULL + (hash << 6) +
            (hash >> 2);
    return hash;
  }

  static double NowSeconds() {
    using clock = std::chrono::steady_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
  }

  std::weak_ptr<Scene> scene_;
  std::string name_;
  Entity entity_{};
  std::shared_ptr<Mesh> mesh_;
  std::shared_ptr<Material> material_;

  mutable std::mutex pending_mutex_;
  PendingPayload pending_;

  std::uint64_t last_published_hash_ = 0;
  std::size_t last_published_vertex_count_ = 0;
  std::size_t last_published_triangle_count_ = 0;
  double last_publish_time_seconds_ = -1.0e18;

  Stats stats_{};
};

}  // namespace l_system_package
