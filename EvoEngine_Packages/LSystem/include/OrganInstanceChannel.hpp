#pragma once

#include "LSystem_PCH.hpp"
#include "RenderPublishPolicy.hpp"

#include "AssetManager.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "Particles.hpp"
#include "Scene.hpp"

namespace l_system_package {
using namespace evo_engine;

class OrganInstanceChannel {
 public:
  struct Stats {
    std::size_t high_water_instance_count = 0;
    std::uint64_t flush_count = 0;
    std::uint64_t skipped_by_dedup = 0;
    std::uint64_t skipped_by_rate_limit = 0;
  };

  OrganInstanceChannel(const std::shared_ptr<Scene>& scene, const Entity& parent, const std::string& name,
                       std::shared_ptr<Mesh> instance_mesh, std::shared_ptr<Material> instance_material)
      : scene_(scene),
        name_(name),
        instance_mesh_(std::move(instance_mesh)),
        instance_material_(std::move(instance_material)) {
    entity_ = scene->CreateEntity(name);
    scene->SetParent(entity_, parent);
    particle_info_list_ = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    particles_ = scene->GetOrSetPrivateComponent<Particles>(entity_).lock();
    particles_->mesh = instance_mesh_;
    particles_->material = instance_material_;
    particles_->particle_info_list = particle_info_list_;
  }

  OrganInstanceChannel(const OrganInstanceChannel&) = delete;
  OrganInstanceChannel& operator=(const OrganInstanceChannel&) = delete;
  OrganInstanceChannel(OrganInstanceChannel&&) = delete;
  OrganInstanceChannel& operator=(OrganInstanceChannel&&) = delete;

  ~OrganInstanceChannel() {
    if (const auto scene = scene_.lock()) {
      if (scene->IsEntityValid(entity_)) {
        scene->DeleteEntity(entity_);
      }
    }
  }

  void Stage(std::vector<ParticleInfo> particle_infos) {
    const std::uint64_t payload_hash = HashBytes(particle_infos.data(), particle_infos.size() * sizeof(ParticleInfo));
    {
      std::lock_guard<std::mutex> guard(pending_mutex_);
      pending_.particle_infos = std::move(particle_infos);
      pending_.hash = payload_hash;
      pending_.present = true;
    }
    if (!policy.defer_to_main_thread) {
      Flush();
    }
  }

  void Publish(std::vector<ParticleInfo> particle_infos) {
    Stage(std::move(particle_infos));
    Flush();
  }

  void Clear() {
    Stage({});
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
        payload.particle_infos.size() == last_published_instance_count_) {
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

    if (!particle_info_list_) {
      return false;
    }
    particle_info_list_->SetParticleInfos(payload.particle_infos);

    last_published_hash_ = payload.hash;
    last_published_instance_count_ = payload.particle_infos.size();
    stats_.high_water_instance_count = std::max(stats_.high_water_instance_count, payload.particle_infos.size());
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
  [[nodiscard]] const std::shared_ptr<Mesh>& GetInstanceMesh() const noexcept {
    return instance_mesh_;
  }
  [[nodiscard]] const std::shared_ptr<Material>& GetInstanceMaterial() const noexcept {
    return instance_material_;
  }
  [[nodiscard]] const std::shared_ptr<ParticleInfoList>& GetParticleInfoList() const noexcept {
    return particle_info_list_;
  }
  [[nodiscard]] const std::string& GetName() const noexcept {
    return name_;
  }
  [[nodiscard]] Stats GetStats() const noexcept {
    return stats_;
  }

  RenderPublishPolicy policy{};

 private:
  struct PendingPayload {
    std::vector<ParticleInfo> particle_infos;
    std::uint64_t hash = 0;
    bool present = false;
  };

  static double NowSeconds() {
    using clock = std::chrono::steady_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
  }

  std::weak_ptr<Scene> scene_;
  std::string name_;
  Entity entity_{};
  std::shared_ptr<Mesh> instance_mesh_;
  std::shared_ptr<Material> instance_material_;
  std::shared_ptr<ParticleInfoList> particle_info_list_;
  std::shared_ptr<Particles> particles_;

  mutable std::mutex pending_mutex_;
  PendingPayload pending_;

  std::uint64_t last_published_hash_ = 0;
  std::size_t last_published_instance_count_ = 0;
  double last_publish_time_seconds_ = -1.0e18;

  Stats stats_{};
};

}  // namespace l_system_package
