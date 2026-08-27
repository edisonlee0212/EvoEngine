
#pragma once

namespace evo_engine {
class EVOENGINE_API File;
class EVOENGINE_API Folder;

/**
 * The "GUID" for all instances in evo_engine that requires a unique identifier for hashing/serialization.
 */
struct EVOENGINE_API Handle {
  friend class IAsset;
  friend struct EntityMetadata;
  friend class Resources;

  /**
   * Default constructor.
   * Allocates a random number to the handle.
   * Note: evo_engine will not handle collisions since the possibility is extremely small and negligible.
   */
  Handle();

  /**
   * Constructor that initializes the handle with a specific value.
   * @param value The specific value to assign to the handle.
   */
  Handle(uint64_t value);

  /**
   * Copy constructor.
   * @param other The Handle instance to copy data from.
   */
  Handle(const Handle &other);

  /**
   * Implicit conversion operator to uint64_t.
   * Allows the Handle instance to be treated as a uint64_t.
   * @return The value of the handle as uint64_t.
   */
  operator uint64_t() {
    return value_;
  }

  /**
   * Const implicit conversion operator to uint64_t.
   * Allows the Handle instance to be treated as a const uint64_t.
   * @return The value of the handle as uint64_t.
   */
  operator const uint64_t() const {
    return value_;
  }

  /**
   * Retrieves the value of the handle.
   * @return The value of the handle as a uint64_t.
   */
  [[nodiscard]] uint64_t GetValue() const {
    return value_;
  }

 private:
  uint64_t value_;  ///< The unique identifier value for this handle.
};

/**
 * An interface for managing handles associated with objects in evo_engine.
 */
class EVOENGINE_API IHandle {
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
  friend class File;
  friend class Folder;
  friend class PrivateComponentStorage;
  friend class PackageRegistrar;

  Handle handle_;  ///< The handle associated with this object.

 public:
  /**
   * Retrieves the handle associated with this object.
   * @return The Handle instance.
   */
  [[nodiscard]] Handle GetHandle() const {
    return handle_;
  }
};

}  // namespace evo_engine

/**
 * Specialization of the std::hash struct for evo_engine::Handle.
 * Provides a hashing function for evo_engine::Handle.
 */
template <>
struct std::hash<evo_engine::Handle> {
  /**
   * Hashing function for evo_engine::Handle.
   * @param handle The handle to generate the hash for.
   * @return The hash value for the given handle.
   */
  size_t operator()(const evo_engine::Handle &handle) const {
    return hash<uint64_t>()(handle);
  }
};
