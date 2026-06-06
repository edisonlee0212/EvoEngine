#pragma once

#include "LSystem_PCH.hpp"
#include "RenderPublishPolicy.hpp"

#include "AssetManager.hpp"
#include "Material.hpp"
#include "Scene.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"

namespace l_system_package {
using namespace evo_engine;

class OrganStrandsChannel {
 public:
  struct Stats {
    std::size_t high_water_point_count = 0;
    std::size_t high_water_index_count = 0;
    std::uint64_t flush_count = 0;
    std::uint64_t skipped_by_dedup = 0;
    std::uint64_t skipped_by_rate_limit = 0;
  };

  OrganStrandsChannel(const std::shared_ptr<Scene>& scene, const Entity& parent, const std::string& name)
      : scene_(scene), name_(name) {
    entity_ = scene->CreateEntity(name);
    scene->SetParent(entity_, parent);
    strands_ = AssetManager::CreateTemporaryAsset<Strands>();
    material_ = AssetManager::CreateTemporaryAsset<Material>();
    const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity_).lock();
    renderer->strands = strands_;
    renderer->material = material_;
  }

  OrganStrandsChannel(const OrganStrandsChannel&) = delete;
  OrganStrandsChannel& operator=(const OrganStrandsChannel&) = delete;
  OrganStrandsChannel(OrganStrandsChannel&&) = delete;
  OrganStrandsChannel& operator=(OrganStrandsChannel&&) = delete;

  ~OrganStrandsChannel() {
    if (const auto scene = scene_.lock()) {
      if (scene->IsEntityValid(entity_)) {
        scene->DeleteEntity(entity_);
      }
    }
  }

  void StageSegments(StrandPointAttributes attributes, std::vector<glm::uint> segments,
                     std::vector<StrandPoint> points) {
    Stage(PublishMode::Segments, attributes, std::move(segments), std::move(points));
  }

  void StageStrands(StrandPointAttributes attributes, std::vector<glm::uint> strands, std::vector<StrandPoint> points) {
    Stage(PublishMode::Strands, attributes, std::move(strands), std::move(points));
  }

  void Clear() {
    StageSegments({}, {}, {});
  }

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
        payload.indices.size() == last_published_index_count_ && payload.points.size() == last_published_point_count_) {
      ++stats_.skipped_by_dedup;
      return false;
    }

    if (policy.min_republish_interval_seconds > 0.0f) {
      const double now = NowSeconds();
      if (now - last_publish_time_seconds_ < static_cast<double>(policy.min_republish_interval_seconds)) {
        std::lock_guard<std::mutex> guard(pending_mutex_);
        if (!pending_.present) {
          pending_ = std::move(payload);
        }
        ++stats_.skipped_by_rate_limit;
        return false;
      }
      last_publish_time_seconds_ = now;
    }

    if (!strands_) {
      return false;
    }
    if (payload.mode == PublishMode::Segments) {
      strands_->SetSegments(payload.attributes, payload.indices, payload.points);
    } else {
      strands_->SetStrands(payload.attributes, payload.indices, payload.points);
    }

    last_published_hash_ = payload.hash;
    last_published_index_count_ = payload.indices.size();
    last_published_point_count_ = payload.points.size();
    stats_.high_water_index_count = std::max(stats_.high_water_index_count, payload.indices.size());
    stats_.high_water_point_count = std::max(stats_.high_water_point_count, payload.points.size());
    ++stats_.flush_count;
    return true;
  }

  [[nodiscard]] bool HasPending() const {
    std::lock_guard<std::mutex> guard(pending_mutex_);
    return pending_.present;
  }

  [[nodiscard]] Entity GetEntity() const noexcept {
    return entity_;
  }
  [[nodiscard]] const std::shared_ptr<Strands>& GetStrands() const noexcept {
    return strands_;
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

  RenderPublishPolicy policy{};

 private:
  enum class PublishMode { Segments, Strands };

  struct PendingPayload {
    PublishMode mode = PublishMode::Segments;
    StrandPointAttributes attributes{};
    std::vector<glm::uint> indices;
    std::vector<StrandPoint> points;
    std::uint64_t hash = 0;
    bool present = false;
  };

  void Stage(const PublishMode mode, StrandPointAttributes attributes, std::vector<glm::uint> indices,
             std::vector<StrandPoint> points) {
    std::uint64_t payload_hash = HashBytes(indices.data(), indices.size() * sizeof(glm::uint));
    payload_hash ^= HashBytes(points.data(), points.size() * sizeof(StrandPoint)) + 0x9e3779b97f4a7c15ULL +
                    (payload_hash << 6) + (payload_hash >> 2);
    {
      std::lock_guard<std::mutex> guard(pending_mutex_);
      pending_.mode = mode;
      pending_.attributes = attributes;
      pending_.indices = std::move(indices);
      pending_.points = std::move(points);
      pending_.hash = payload_hash;
      pending_.present = true;
    }
    if (!policy.defer_to_main_thread) {
      Flush();
    }
  }

  static double NowSeconds() {
    using clock = std::chrono::steady_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
  }

  std::weak_ptr<Scene> scene_;
  std::string name_;
  Entity entity_{};
  std::shared_ptr<Strands> strands_;
  std::shared_ptr<Material> material_;

  mutable std::mutex pending_mutex_;
  PendingPayload pending_;

  std::uint64_t last_published_hash_ = 0;
  std::size_t last_published_index_count_ = 0;
  std::size_t last_published_point_count_ = 0;
  double last_publish_time_seconds_ = -1.0e18;

  Stats stats_{};
};

}  // namespace l_system_package
