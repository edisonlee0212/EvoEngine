#pragma once

#include "Entity.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace evo_engine {

class EVOENGINE_API Scene;

class EVOENGINE_API EntitySelection final {
 public:
  enum class RequestSource : uint8_t { User, Programmatic, Lifecycle };
  enum class AnchorPolicy : uint8_t { Preserve, Set, Clear };
  enum class Result : uint8_t { Changed, Unchanged, RejectedLocked, RejectedInvalid };

  struct RequestOptions {
    RequestSource source = RequestSource::User;
    AnchorPolicy anchor_policy = AnchorPolicy::Preserve;
    Entity anchor{};
  };

  struct EVOENGINE_API Snapshot {
    std::weak_ptr<Scene> scene;
    std::vector<Entity> entities;
    Entity primary{};
    Entity anchor{};
    bool locked = false;
    uint64_t revision = 0;

    [[nodiscard]] bool Contains(const Entity& entity) const;
  };

  Result BindScene(const std::shared_ptr<Scene>& scene);
  Result Replace(const Entity& entity);
  Result Replace(const Entity& entity, const RequestOptions& options);
  Result ReplaceMany(const std::vector<Entity>& entities, const Entity& primary);
  Result ReplaceMany(const std::vector<Entity>& entities, const Entity& primary, const RequestOptions& options);
  Result Add(const Entity& entity);
  Result Add(const Entity& entity, const RequestOptions& options);
  Result AddMany(const std::vector<Entity>& entities, const Entity& primary);
  Result AddMany(const std::vector<Entity>& entities, const Entity& primary, const RequestOptions& options);
  Result Remove(const Entity& entity);
  Result Remove(const Entity& entity, const RequestOptions& options);
  Result Toggle(const Entity& entity);
  Result Toggle(const Entity& entity, const RequestOptions& options);
  Result Clear(RequestSource source = RequestSource::User);
  Result ClearAnchor(RequestSource source = RequestSource::User);
  Result PruneInvalid();
  Result SetLocked(bool locked, RequestSource source = RequestSource::User);

  [[nodiscard]] const std::vector<Entity>& GetEntities() const;
  [[nodiscard]] Entity GetPrimary() const;
  [[nodiscard]] Entity GetAnchor() const;
  [[nodiscard]] bool Contains(const Entity& entity) const;
  [[nodiscard]] size_t GetCount() const;
  [[nodiscard]] bool Empty() const;
  [[nodiscard]] bool IsLocked() const;
  [[nodiscard]] uint64_t GetRevision() const;
  [[nodiscard]] Snapshot GetSnapshot() const;

 private:
  [[nodiscard]] bool IsRequestLocked(RequestSource source) const;
  [[nodiscard]] bool IsValid(const Entity& entity) const;
  [[nodiscard]] Result ValidateEntities(const std::vector<Entity>& entities, const Entity& primary) const;
  bool ApplyAnchor(const RequestOptions& options, const Entity& fallback);
  void PromoteMostRecent();
  void IncrementRevision();

  std::weak_ptr<Scene> scene_;
  std::vector<Entity> entities_;
  Entity primary_{};
  Entity anchor_{};
  bool locked_ = false;
  uint64_t revision_ = 0;
};

}  // namespace evo_engine
