#pragma once
#include "TreeControllers.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * \class IShootDescriptor
 * \brief Represents the parameters controlling procedural tree growth.
 */
class IShootDescriptor : public IAsset {
 public:
  /**
   * \brief Prepares a ShootGrowthController using current growth parameters.
   * \param shoot_growth_controller The controller to configure.
   */
  virtual void PrepareController(ShootGrowthController& shoot_growth_controller) const = 0;
  /**
   * \brief Generates a thumbnail texture representing the shoot descriptor.
   * \return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};

/**
 * \class IRootDescriptor
 * \brief Represents the parameters controlling procedural tree growth.
 */
class IRootDescriptor : public IAsset {
 public:
  /**
   * \brief Prepares a RootGrowthController using current growth parameters.
   * \param root_growth_controller The controller to configure.
   */
  virtual void PrepareController(RootGrowthController& root_growth_controller) const = 0;
  /**
   * \brief Generates a thumbnail texture representing the root descriptor.
   * \return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};

/**
 * \class IRootDescriptor
 * \brief Represents the parameters controlling procedural tree growth.
 */
class IFineRootDescriptor : public IAsset {
 public:
  /**
   * \brief Prepares a FineRootDescriptor using current growth parameters.
   * \param fine_root_controller The controller to configure.
   */
  virtual void PrepareController(FineRootController& fine_root_controller) const = 0;
  /**
   * \brief Generates a thumbnail texture representing the root descriptor.
   * \return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};

/**
 * \class IPruningDescriptor
 * \brief Represents the parameters controlling procedural tree pruning.
 */
class IPruningDescriptor : public IAsset {
 public:
  /**
   * \brief Prepares a ShootPruningController using current growth parameters.
   * \param simulation_settings Simulation settings.
   * \param shoot_pruning_controller The controller to configure.
   */
  virtual void PrepareController(const SimulationSettings& simulation_settings,
                                 ShootPruningController& shoot_pruning_controller) const = 0;
  /**
   * \brief Generates a thumbnail texture representing the shoot descriptor.
   * \return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};

/**
 * \class IFoliageDescriptor
 * \brief Represents the parameters controlling foliage generation.
 */
class IFoliageDescriptor : public IAsset {
 public:
  /**
   * \brief Prepares a ShootGrowthController using current growth parameters.
   * \param foliage_controller The controller to configure.
   */
  virtual void PrepareController(FoliageController& foliage_controller) const = 0;
  /**
   * @brief Generates foliage transformation matrices based on internode information.
   * @param[out] matrices Vector to store the transformation matrices.
   * @param[in] internode_info Information about the skeleton node internode.
   * @param[in] tree_size The overall tree size.
   */
  virtual void GenerateFoliageMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internode_info,
                                       float tree_size) const = 0;
  /**
   * \brief Generates a thumbnail texture representing the shoot descriptor.
   * \return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};

/**
 * \class IReproductionModuleDescriptor
 * \brief Represents the parameters controlling fruit generation.
 */
class IReproductionModuleDescriptor : public IAsset {
 public:
  /**
   * \brief Prepares a ShootGrowthController using current growth parameters.
   * \param reproduction_controller The controller to configure.
   */
  virtual void PrepareController(ShootReproductionController& reproduction_controller) const = 0;
  /**
   * @brief Generates fruit transformation matrices based on internode information.
   * @param[out] matrices Vector to store the transformation matrices.
   * @param[in] internode_info Information about the skeleton node internode.
   * @param[in] tree_size The overall tree size.
   */
  virtual void GenerateFruitMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internode_info,
                                     float tree_size) const = 0;
  /**
   * \brief Generates a thumbnail texture representing the shoot descriptor.
   * \return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};

/**
 * \class IBarkDescriptor
 * \brief Represents the parameters controlling foliage generation.
 */
class IBarkDescriptor : public IAsset {
 public:
  /**
   * @brief Computes a bark pattern value based on input parameters.
   * @param x_factor A factor affecting the bark pattern along the X-axis.
   * @param distance_to_root Distance from the root of the tree.
   * @return The computed bark pattern value.
   */
  virtual float GetValue(float x_factor, float distance_to_root) const = 0;
  /**
   * \brief Generates a thumbnail texture representing the shoot descriptor.
   * \return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};

/**
 * \class IFlowerDescriptor
 * \brief Represents the parameters controlling foliage generation.
 */
class IFlowerDescriptor : public IAsset {
 public:
  /**
   * \brief Generates a thumbnail texture representing the shoot descriptor.
   * \return A shared pointer to the generated Texture2D.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};

/**
 * @brief Represents a tree descriptor asset in the EcoSysLab package.
 *
 * This class contains references to various assets required for tree generation,
 * including shoot, foliage, fruit, flower, and bark descriptors. It provides
 * serialization, deserialization, and instantiation functionalities.
 */
class TreeDescriptor : public IAsset {
 public:
  /**
   * @brief Reference to the shoot descriptor asset.
   */
  AssetRef shoot_descriptor;
  /**
   * @brief Reference to the root descriptor asset.
   */
  AssetRef root_descriptor;
  /**
   * @brief Reference to the root descriptor asset.
   */
  AssetRef fine_root_descriptor;
  /**
   * @brief Reference to the pruning descriptor asset.
   */
  AssetRef pruning_descriptor;
  /**
   * @brief Reference to the foliage descriptor asset.
   */
  AssetRef foliage_descriptor;

  /**
   * @brief Reference to the fruit descriptor asset.
   */
  AssetRef reproduction_module_descriptor;

  /**
   * @brief Reference to the bark descriptor asset.
   */
  AssetRef bark_descriptor;

  /**
   * @brief Called when the asset is created.
   */
  void OnCreate() override;

  /**
   * @brief Inspects the asset in the editor.
   *
   * This function will be called by the editor layer to inspect the asset's properties.
   *
   * @param editor_layer The shared pointer to the editor layer.
   * @return Returns true if the asset's content is not modified during inspection.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Collects all asset references contained within this asset.
   *
   * This function populates the provided list with asset references used in this descriptor.
   *
   * @param list A vector to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Instantiates the tree entity from this descriptor.
   *
   * @return The instantiated tree entity.
   */
  Entity Instantiate() const;

  /**
   * @brief Generates a thumbnail texture for the asset.
   *
   * @return A shared pointer to a Texture2D representing the thumbnail.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
};
}  // namespace eco_sys_lab_package
