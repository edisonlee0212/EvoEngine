#pragma once

#include "Skeleton.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @struct TreeGraphNode
 * @brief Represents a node in the tree graph.
 */
struct TreeGraphNode {
  glm::vec3 start;                                       ///< The starting position of the node.
  float length;                                          ///< The length of the node.
  float thickness;                                       ///< The thickness of the node.
  int id;                                                ///< The unique identifier for the node.
  int parent_id;                                         ///< The unique identifier of the parent node.
  bool from_apical_bud = false;                          ///< Indicates if the node originates from an apical bud.
  glm::quat global_rotation;                             ///< The global rotation of the node.
  glm::vec3 position;                                    ///< The position of the node.
  std::weak_ptr<TreeGraphNode> parent;                   ///< A weak reference to the parent node.
  std::vector<std::shared_ptr<TreeGraphNode>> children;  ///< A list of child nodes.
};

/**
 * @class TreeGraph
 * @brief Represents a hierarchical tree data structure.
 */
class TreeGraph : public IAsset {
  /**
   * @brief Recursively collects child nodes for constructing the tree graph.
   * @param node The current node being processed.
   * @param graph_nodes The resulting collection of nodes grouped by layers.
   * @param current_layer The current processing layer index.
   */
  void CollectChild(const std::shared_ptr<TreeGraphNode>& node,
                    std::vector<std::vector<std::shared_ptr<TreeGraphNode>>>& graph_nodes, int current_layer) const;

 public:
  bool enable_instantiate_length_limit = false;  ///< Enables or disables length limits for instantiation.
  float instantiate_length_limit = 8.0f;         ///< The maximum permissible length for instantiation.
  std::shared_ptr<TreeGraphNode> root;           ///< The root node of the tree graph.
  std::string name;                              ///< The name of the tree graph.
  int layer_size;                                ///< The number of layers in the tree graph.

  /**
   * @brief Collects asset references associated with this asset.
   * @param list Output list of asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Serializes the tree graph data into a YAML output stream.
   * @param out The YAML emitter that stores the serialized data.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes tree graph data from a YAML input node.
   * @param in The YAML node containing the deserialized data.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Handles the inspection of the asset in the editor.
   * @param editor_layer The editor layer that requests inspection.
   * @return True if the asset content remains unchanged; otherwise, false.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};

/**
 * @class TreeGraphV2
 * @brief A second version of the hierarchical tree data structure.
 */
class TreeGraphV2 : public IAsset {
  /**
   * @brief Recursively collects child nodes for constructing the tree graph.
   * @param node The current node being processed.
   * @param graph_nodes The resulting collection of nodes grouped by layers.
   * @param current_layer The current processing layer index.
   */
  void CollectChild(const std::shared_ptr<TreeGraphNode>& node,
                    std::vector<std::vector<std::shared_ptr<TreeGraphNode>>>& graph_nodes, int current_layer) const;

 public:
  bool enable_instantiate_length_limit = false;  ///< Enables or disables length limits for instantiation.
  float instantiate_length_limit = 8.0f;         ///< The maximum permissible length for instantiation.
  std::shared_ptr<TreeGraphNode> m_root;         ///< The root node of the tree graph.
  std::string name;                              ///< The name of the tree graph.
  int layer_size;                                ///< The number of layers in the tree graph.

  /**
   * @brief Collects asset references associated with this asset.
   * @param list Output list of asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * @brief Serializes the tree graph data into a YAML output stream.
   * @param out The YAML emitter that stores the serialized data.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes tree graph data from a YAML input node.
   * @param in The YAML node containing the deserialized data.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Handles the inspection of the asset in the editor.
   * @param editor_layer The editor layer that requests inspection.
   * @return True if the asset content remains unchanged; otherwise, false.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};
}  // namespace eco_sys_lab_package