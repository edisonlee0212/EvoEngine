#include "EntitySelection.hpp"

#include "Scene.hpp"

#include <algorithm>

using namespace evo_engine;

bool EntitySelection::Snapshot::Contains(const Entity& entity) const {
  return std::find(entities.begin(), entities.end(), entity) != entities.end();
}

EntitySelection::Result EntitySelection::BindScene(const std::shared_ptr<Scene>& scene) {
  if (scene_.lock() == scene) {
    return Result::Unchanged;
  }
  scene_ = scene;
  entities_.clear();
  primary_ = {};
  anchor_ = {};
  locked_ = false;
  IncrementRevision();
  return Result::Changed;
}

EntitySelection::Result EntitySelection::Replace(const Entity& entity) {
  return Replace(entity, {});
}

EntitySelection::Result EntitySelection::Replace(const Entity& entity, const RequestOptions& options) {
  if (entity.GetIndex() == 0) {
    return Clear(options.source);
  }
  return ReplaceMany({entity}, entity, options);
}

EntitySelection::Result EntitySelection::ReplaceMany(const std::vector<Entity>& entities, const Entity& primary) {
  return ReplaceMany(entities, primary, {});
}

EntitySelection::Result EntitySelection::ReplaceMany(const std::vector<Entity>& entities, const Entity& primary,
                                                     const RequestOptions& options) {
  if (IsRequestLocked(options.source)) {
    return Result::RejectedLocked;
  }
  if (entities.empty()) {
    return Clear(options.source);
  }
  if (const auto result = ValidateEntities(entities, primary); result != Result::Unchanged) {
    return result;
  }

  std::vector<Entity> replacement;
  replacement.reserve(entities.size());
  for (const auto& entity : entities) {
    if (std::find(replacement.begin(), replacement.end(), entity) == replacement.end() && entity != primary) {
      replacement.emplace_back(entity);
    }
  }
  replacement.emplace_back(primary);

  const auto previous_entities = entities_;
  const auto previous_primary = primary_;
  const auto previous_anchor = anchor_;
  entities_ = std::move(replacement);
  primary_ = primary;
  ApplyAnchor(options, primary);
  if (entities_ == previous_entities && primary_ == previous_primary && anchor_ == previous_anchor) {
    return Result::Unchanged;
  }
  IncrementRevision();
  return Result::Changed;
}

EntitySelection::Result EntitySelection::Add(const Entity& entity) {
  return Add(entity, {});
}

EntitySelection::Result EntitySelection::Add(const Entity& entity, const RequestOptions& options) {
  return AddMany({entity}, entity, options);
}

EntitySelection::Result EntitySelection::AddMany(const std::vector<Entity>& entities, const Entity& primary) {
  return AddMany(entities, primary, {});
}

EntitySelection::Result EntitySelection::AddMany(const std::vector<Entity>& entities, const Entity& primary,
                                                 const RequestOptions& options) {
  if (IsRequestLocked(options.source)) {
    return Result::RejectedLocked;
  }
  if (entities.empty()) {
    const auto previous_anchor = anchor_;
    ApplyAnchor(options, {});
    if (anchor_ == previous_anchor) {
      return Result::Unchanged;
    }
    IncrementRevision();
    return Result::Changed;
  }
  if (const auto result = ValidateEntities(entities, primary); result != Result::Unchanged) {
    return result;
  }

  const auto previous_anchor = anchor_;
  bool added = false;
  for (const auto& entity : entities) {
    if (!Contains(entity)) {
      entities_.emplace_back(entity);
      added = true;
    }
  }
  if (added) {
    entities_.erase(std::remove(entities_.begin(), entities_.end(), primary), entities_.end());
    entities_.emplace_back(primary);
    primary_ = primary;
  }
  ApplyAnchor(options, primary);
  if (!added && anchor_ == previous_anchor) {
    return Result::Unchanged;
  }
  IncrementRevision();
  return Result::Changed;
}

EntitySelection::Result EntitySelection::Remove(const Entity& entity) {
  return Remove(entity, {});
}

EntitySelection::Result EntitySelection::Remove(const Entity& entity, const RequestOptions& options) {
  if (IsRequestLocked(options.source)) {
    return Result::RejectedLocked;
  }
  const auto found = std::find(entities_.begin(), entities_.end(), entity);
  if (found == entities_.end()) {
    const auto previous_anchor = anchor_;
    ApplyAnchor(options, primary_);
    if (anchor_ == previous_anchor) {
      return Result::Unchanged;
    }
    IncrementRevision();
    return Result::Changed;
  }

  const bool removed_primary = primary_ == entity;
  const bool removed_anchor = anchor_ == entity;
  entities_.erase(found);
  if (entities_.empty()) {
    primary_ = {};
    anchor_ = {};
    locked_ = false;
  } else {
    if (removed_primary) {
      PromoteMostRecent();
    }
    if (removed_anchor) {
      anchor_ = primary_;
    }
    ApplyAnchor(options, primary_);
  }
  IncrementRevision();
  return Result::Changed;
}

EntitySelection::Result EntitySelection::Toggle(const Entity& entity) {
  return Toggle(entity, {});
}

EntitySelection::Result EntitySelection::Toggle(const Entity& entity, const RequestOptions& options) {
  return Contains(entity) ? Remove(entity, options) : Add(entity, options);
}

EntitySelection::Result EntitySelection::Clear(const RequestSource) {
  if (entities_.empty() && primary_.GetIndex() == 0 && anchor_.GetIndex() == 0 && !locked_) {
    return Result::Unchanged;
  }
  entities_.clear();
  primary_ = {};
  anchor_ = {};
  locked_ = false;
  IncrementRevision();
  return Result::Changed;
}

EntitySelection::Result EntitySelection::ClearAnchor(const RequestSource source) {
  if (IsRequestLocked(source))
    return Result::RejectedLocked;
  if (anchor_.GetIndex() == 0)
    return Result::Unchanged;
  anchor_ = {};
  IncrementRevision();
  return Result::Changed;
}

EntitySelection::Result EntitySelection::PruneInvalid() {
  const auto previous_entities = entities_;
  const auto previous_primary = primary_;
  const auto previous_anchor = anchor_;
  const auto previous_locked = locked_;
  entities_.erase(std::remove_if(entities_.begin(), entities_.end(),
                                 [&](const Entity& entity) {
                                   return !IsValid(entity);
                                 }),
                  entities_.end());
  if (entities_.empty()) {
    primary_ = {};
    anchor_ = {};
    locked_ = false;
  } else {
    if (!Contains(primary_)) {
      PromoteMostRecent();
    }
    if (!Contains(anchor_)) {
      anchor_ = primary_;
    }
  }
  if (entities_ == previous_entities && primary_ == previous_primary && anchor_ == previous_anchor &&
      locked_ == previous_locked) {
    return Result::Unchanged;
  }
  IncrementRevision();
  return Result::Changed;
}

EntitySelection::Result EntitySelection::SetLocked(const bool locked, const RequestSource source) {
  static_cast<void>(source);
  const bool value = locked && !entities_.empty();
  if (locked_ == value) {
    return Result::Unchanged;
  }
  locked_ = value;
  IncrementRevision();
  return Result::Changed;
}

const std::vector<Entity>& EntitySelection::GetEntities() const {
  return entities_;
}

Entity EntitySelection::GetPrimary() const {
  return primary_;
}

Entity EntitySelection::GetAnchor() const {
  return anchor_;
}

bool EntitySelection::Contains(const Entity& entity) const {
  return std::find(entities_.begin(), entities_.end(), entity) != entities_.end();
}

size_t EntitySelection::GetCount() const {
  return entities_.size();
}

bool EntitySelection::Empty() const {
  return entities_.empty();
}

bool EntitySelection::IsLocked() const {
  return locked_;
}

uint64_t EntitySelection::GetRevision() const {
  return revision_;
}

EntitySelection::Snapshot EntitySelection::GetSnapshot() const {
  return {scene_, entities_, primary_, anchor_, locked_, revision_};
}

bool EntitySelection::IsRequestLocked(const RequestSource source) const {
  return locked_ && source == RequestSource::User;
}

bool EntitySelection::IsValid(const Entity& entity) const {
  const auto scene = scene_.lock();
  return scene && scene->IsEntityValid(entity);
}

EntitySelection::Result EntitySelection::ValidateEntities(const std::vector<Entity>& entities,
                                                          const Entity& primary) const {
  if (!IsValid(primary)) {
    return Result::RejectedInvalid;
  }
  for (const auto& entity : entities) {
    if (!IsValid(entity)) {
      return Result::RejectedInvalid;
    }
  }
  return Result::Unchanged;
}

bool EntitySelection::ApplyAnchor(const RequestOptions& options, const Entity& fallback) {
  const auto previous = anchor_;
  switch (options.anchor_policy) {
    case AnchorPolicy::Preserve:
      break;
    case AnchorPolicy::Set:
      anchor_ = IsValid(options.anchor) && Contains(options.anchor) ? options.anchor : fallback;
      break;
    case AnchorPolicy::Clear:
      anchor_ = {};
      break;
  }
  return anchor_ != previous;
}

void EntitySelection::PromoteMostRecent() {
  primary_ = entities_.empty() ? Entity{} : entities_.back();
}

void EntitySelection::IncrementRevision() {
  ++revision_;
}
