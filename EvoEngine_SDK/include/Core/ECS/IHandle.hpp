#pragma once
namespace evo_engine {
/**
 * The "GUID" for all instances in evo_engine that requires a unique identifier for hashing/serialization.
 */
struct Handle {
  friend class IAsset;
  friend struct EntityMetadata;
  friend class Resources;

  /**
   * Default constructor, will allocate a random number to the handle. evo_engine will not handle the collision since
   * possibility is extremely small and negligible.
   */
  Handle();
  /**
   * Constructor that takes a value for the handle.
   */
  Handle(uint64_t value);
  Handle(const Handle &other);

  operator uint64_t() {
    return value_;
  }
  operator const uint64_t() const {
    return value_;
  }
  [[nodiscard]] uint64_t GetValue() const {
    return value_;
  }

 private:
  uint64_t value_;
};
class IHandle {
  friend class Prefab;
  friend class Entities;
  friend struct EntityMetadata;

  friend class EditorLayer;
  friend class Resources;
  friend class Serialization;
  friend class IAsset;
  friend class AssetRef;
  friend class PrivateComponentRef;
  friend class Scene;
  friend class AssetRecord;
  friend class Folder;
  friend class PrivateComponentStorage;
  Handle handle_;

 public:
  [[nodiscard]] Handle GetHandle() const {
    return handle_;
  }
};
}  // namespace evo_engine

template <>
struct std::hash<evo_engine::Handle> {
  size_t operator()(const evo_engine::Handle &handle) const {
    return hash<uint64_t>()(handle);
  }
};  // namespace std