/**
 * @author David Hrusa, Tomas Polasek
 * @date 11.20.2019
 * @version 1.0
 * @brief Simple module for representing importing and exporting skeleton files .tree (implements 1.0 .tree encoding).
 */

#ifndef TREEIO_TREE
#define TREEIO_TREE

#include "TreeIOUtils.hpp"

namespace treeio {

/// @brief Data contained within a single TreeNode.
struct TreeNodeData {
  /// @brief Swaps the values between dira and dirb (x - 1, y - 2, z - 3) Negative number means opposite direction
  void swapNodeCoords(int dira, int dirb);

  /// Position of this node in model-space.
  treeutil::Vector3D pos{};
  /// Optional thickness override.
  float thickness{0.0f};
  /// Freeze thickness to the pre-set value?
  bool freezeThickness{false};

  /// Tangent to the branch.
  treeutil::Vector3D tangent{};
  /// Direction of the branch.
  treeutil::Vector3D direction{};
  /// Parallel of the branch.
  treeutil::Vector3D vParallel{};
  /// How far from start of the chain is this node.
  std::size_t distanceFromChainStart{0u};
  /// Thickness of parent node (optional).
  float parentThickness{0.0f};
  /// Order of this node in sorted sequence.
  std::size_t order{0u};

  /// Color of the line.
  treeutil::Color lineColor{1.0f, 0.0f, 0.0f, 1.0f};
  /// Color of the node.
  treeutil::Color pointColor{1.0f, 1.0f, 0.0f, 1.0f};
};  // struct TreeNodeData

struct TreeMetaData;

/**
 * @brief Holds meta data which is only relevant while the tree is loaded. Also provides a method called at load and
 *  save to fill the data structure and/or flush the data back into the main TreeMetaData.
 */
struct TreeRuntimeMetaData {
  /// @brief Pointer to general runtime metadata.
  using Ptr = treeutil::WrapperPtrT<TreeRuntimeMetaData>;

  /// @brief Create a duplicate of this runtime meta-data structure.
  virtual Ptr duplicate() const = 0;

  /// @brief called during loading of the tree to refresh the runtime metadata after deserialization.
  virtual void onLoad(TreeMetaData &metaData) = 0;

  /// @brief called during saving of the tree to store any runtime variables into the permanent meta data prior to
  /// serialization.
  virtual void onSave(TreeMetaData &metaData) = 0;
};  // struct TreeRuntimeMetaData

/// @brief Dynamic meta-data represented by associative container.
struct TreeDynamicMetaData {
  /// Tag signalling start of serialized dynamic meta-data.
  static constexpr auto DYNAMIC_DATA_START_TAG{"<DynamicData>"};
  /// Tag signalling end of serialized dynamic meta-data.
  static constexpr auto DYNAMIC_DATA_END_TAG{"</DynamicData>"};

  /// @brief Serialize all dynamic meta-data.
  std::string serialize() const;

  /// @brief Load serialized dynamic meta-datafrom given string.
  void deserialize(const std::string &serialized);

  /// Internal data holder.
  json data{};
};  // struct TreeDynamicMetaData

/// @brief Holds meta data containing information regarding the tree
struct TreeMetaData {
  /// @brief Initialize tree meta-data.
  TreeMetaData();
  /// @brief Clean up and destroy.
  ~TreeMetaData();

  /// @brief Serialize all meta-data.
  std::string serialize() const;
  /// @brief De-serialize all meta-data and load them to this instance.
  void deserialize(const std::string &serialized, const std::shared_ptr<TreeRuntimeMetaData> &runtime);

  /// @brief Insert value to this meta-data instance by key name.
  void insertValue(const std::string &key, const std::string &value);

  /// @brief Set invalid values to correct state.
  void validateValues();

  /// @brief Calculate complete scale of the skeleton in meters.
  float calcSkeletonScale() const {
    return skeletonScale * baseScale;
  }

  /// @brief Calculate complete scale of the reference in meters.
  float calcReferenceScale() const {
    return referenceScale * baseScale;
  }

  /// @brief Calculate complete scale of the reconstruction in meters.
  float calcReconstructionScale() const {
    return reconstructionScale * baseScale;
  }

  // Pseudo info (depends on the users):
  /// Numbering from your database
  std::size_t treeID = 0;
  /// Description of what the tree looks like in words (not a comparable parameter)
  std::string infoText;
  /// Holds batching scenario data which can be stored or loaded or invoked by other batch scripts.
  std::string batchScenario;

  // Objective source / type info
  /// Version of .tree encoding used
  std::string version{"1.0"};
  /// Name of the tree in format tree#.
  std::string treeName;
  /// Path to the reference model used.
  std::string treeReference;
  /// Extension of the input model - obj / txtslk / skeleton.
  std::string extension;
  /// Source of the input model - Online / InvTree / ShapeSpace / TreeParts / SpeedTree.
  std::string source;
  /// Type of the input model - Model / PointSkeleton.
  std::string modelType;
  /// Style of the input model - Synthetic / Real.
  std::string style;
  /// Character of the input model - Deciduous / Coniferous / Palm.
  std::string character;
  /// Did this model need processing?
  bool reqProcess{true};
  /// Is this model processed?
  bool processed{true};
  /// Is this model skeletonized?
  bool skeletonized{false};
  /// Is this model completely finished, including all information and skeleton?
  bool finalized{false};

  /// Age estimate in years.
  float ageEstimate{0.0f};
  /// Distance of internodes in normalized units.
  float internodalDistance{0.0f};

  /// Scale of the skeleton to 1.0 cube.
  float skeletonScale{1.0f};
  /// Scale of the reference model to 1.0 cube.
  float referenceScale{-1.0f};
  /// Scale of the reconstructed model to 1.0 cube.
  float reconstructionScale{-1.0f};
  /// Scale of the tree into real meters.
  float baseScale{1.0f};

  /// How thickness scales up with intersections
  float thicknessFactor{1.6f};
  /// Starting thickness of the smallest branches.
  float startingThickness{0.02f};
  /// Tension of branches connecting skeleton nodes.
  float branchTension{0.0f};
  /// Bias of branches connecting skeleton nodes.
  float branchBias{0.0f};
  /// Multiplier used for branch thickness.
  float branchWidthMultiplier{1.0f};
  /// Are we using the old branch width multiplier?
  bool oldBranchWidthMultiplier{true};
  /// Recalculate radius of the branches using factor and thickness?
  bool recalculateRadius{false};

  // TODO - calculate branching number

  // Implementation info (used by various other applications):
  /// Angle under which the tree has the most interesting silhouette.
  float distinctAngle{0.0f};
  /// Has the tree skeleton been decimated?
  bool decimated{false};
  /// Epsilon used in the decimation.
  float decimationEpsilon{0.0f};

  /// @brief typecasts the pointer to your particular implementation for convenience.
  template <typename T>
  std::shared_ptr<T> getRuntimeMetaData() {
    return mRuntimeMetaData ? std::dynamic_pointer_cast<T>(mRuntimeMetaData.ptr) : std::shared_ptr<T>{nullptr};
  }

  /// @brief Set runtime meta-data for this object, automatically initializing it.
  void setRuntimeMetaData(const std::shared_ptr<TreeRuntimeMetaData> &runtime) {
    mRuntimeMetaData = runtime;
    onLoad();
  }

  /// @brief Access the dynamic meta-data.
  const json &dynamicData() const {
    return mDynamicMetaData->data;
  }

  /// @brief Access the dynamic meta-data.
  json &dynamicData() {
    return mDynamicMetaData->data;
  }

 private:
  /// @brief call after the metadata is deserialized.
  void onLoad();

  /// @brief call before the metadata is serialized.
  void onSave();

  /// Runtime only meta-data.
  treeutil::CopyableSmartPtr<TreeRuntimeMetaData, std::shared_ptr> mRuntimeMetaData{nullptr};

  /// Dynamic meta-data represented by associative container.
  treeutil::CopyableSmartPtr<TreeDynamicMetaData, std::shared_ptr> mDynamicMetaData{nullptr};

 protected:
};  // struct TreeMetaData

/// @brief A single node within the tree.
template <typename InternalDataT>
class TreeNodeT {
 public:
  /// @brief Type of the internal data stored within this node.
  using NodeDataT = InternalDataT;

  /// @brief Type used to store nodes and other data.
  template <typename T>
  using NodeArrayT = std::vector<T>;
  /// @brief Type used for indexing tree nodes.
  using NodeIdT = std::size_t;
  /// @brief Type used to store nodes children.
  using NodeChildArrayT = NodeArrayT<NodeIdT>;
  /// Value representing invalid index to a node.
  static constexpr auto INVALID_NODE_ID{NodeIdT{0u}};

  /// @brief Create an invalid tree node.
  TreeNodeT();
  /// @brief Create tree node with provided data.
  explicit TreeNodeT(const NodeDataT &data);

  /// @brief Create a copy of this node with different internal node data type.
  template <typename OutDataT>
  TreeNodeT<OutDataT> copy() const;

  /// @brief Get index of nodes parent.
  const NodeIdT &parent() const;
  /// @brief Get index of this node.
  const NodeIdT &id() const;
  /// @brief Get list of nodes children.
  const NodeArrayT<NodeIdT> &children() const;

  /// @brief Access data contained within this node.
  NodeDataT &data();
  /// @brief Access data contained within this node.
  const NodeDataT &data() const;

 private:
  // Allow access to internals.
  template <typename DataT, typename MetaDataT>
  friend class ArrayTreeT;
  template <typename T>
  friend class TreeNodeT;

  /// @brief Get list of nodes children.
  NodeArrayT<NodeIdT> &children();

  /// Index of the parent node.
  NodeIdT mArrayParent{INVALID_NODE_ID};
  /// Index of this node within the tree.
  NodeIdT mArrayId{INVALID_NODE_ID};
  /// Array containing indices of child nodes.
  NodeArrayT<NodeIdT> mArrayChildren{};
  /// Data contained within this node.
  NodeDataT mNodeData{};

 protected:
};  // struct TreeNodeT

/// @brief Shortcut for the normally used TreeNode.
using TreeNode = TreeNodeT<TreeNodeData>;

// Forward declaration of the iterator.
template <typename AT, bool ConstIterator, typename UserDataT>
class ArrayTreeIteratorT;

/**
 * @brief Tree consisting of nodes stored in array type data structure.
 * @tparam NDT Type of the internal node data.
 * @tparam MDT Type of the meta-data.
 */
template <typename NDT, typename MDT>
class ArrayTreeT {
 public:
  /// @brief Exception thrown when invalid node identifier is provided.
  struct InvalidNodeIdException : public std::runtime_error {
    InvalidNodeIdException(const char *msg) : std::runtime_error(msg) {
    }
  };  // struct InvalidNodeIdException

  /// @brief This type.
  using ThisT = ArrayTreeT<NDT, MDT>;

  /// @brief Type used to store tree meta-date.
  using MetaDataT = MDT;
  /// @brief Type used as node in the tree.
  using NodeT = TreeNodeT<NDT>;
  /// @brief Type used to store data in the nodes.
  using NodeDataT = typename NodeT::NodeDataT;
  /// @brief Type used to index tree nodes.
  using NodeIdT = typename NodeT::NodeIdT;
  /// @brief Type used to store the nodes.
  template <typename T>
  using NodeArrayT = typename NodeT::template NodeArrayT<T>;
  /// Value representing invalid index to a node.
  static constexpr auto INVALID_NODE_ID{NodeT::INVALID_NODE_ID};
  /// @brief Type used for storing information about node identifier translation.
  using TranslationMapT = std::map<NodeIdT, NodeIdT>;
  /// @brief Type used by nodes to store its child indices.
  using NodeChildArrayT = typename NodeT::NodeChildArrayT;

  // Iterator requirements:
  template <bool ConstIterator, typename UserDataT>
  using IteratorT = ArrayTreeIteratorT<ThisT, ConstIterator, UserDataT>;
  using iterator = IteratorT<false, void>;
  using const_iterator = IteratorT<true, void>;
  using reverse_iterator = iterator;
  using const_reverse_iterator = const_iterator;

  /// @brief Iteration operation styles available.
  enum class IterationStyle {
    // Depth-first iteration going parent -> children.
    DepthFirstChildren,
    // Breadth-first iteration going parent -> children.
    BreadthFirstChildren,
    // Depth-first iteration going child -> parent.
    DepthFirstParents,
    // Breadth-first iteration going child -> parent.
    BreadthFirstParents
  };  // enum class IterationStyle

  /// @brief Create an empty tree.
  ArrayTreeT();
  /// @brief Free all data and destroy.
  ~ArrayTreeT();

  /// @brief Create a copy of this tree with different internal node data type.
  template <typename OutDataT>
  ArrayTreeT<OutDataT, MetaDataT> copy() const;

  // Copy and move operators:
  ArrayTreeT(const ArrayTreeT &tree) = default;
  ArrayTreeT &operator=(const ArrayTreeT &tree) = default;
  ArrayTreeT(ArrayTreeT &&tree) = default;
  ArrayTreeT &operator=(ArrayTreeT &&tree) = default;

  /// @brief Parse ArrayTreeT from string in .tree format.
  template <typename RuntimeMetaDataT>
  static ArrayTreeT fromString(const std::string &serialized);
  /// @brief Parse ArrayTreeT from file path using .tree format.
  template <typename RuntimeMetaDataT>
  static ArrayTreeT fromPath(const std::string &path);

  /// @brief Parse ArrayTreeT from string in .tree format. Does not provide runtime meta-data.
  inline static ArrayTreeT fromStringNoRuntime(const std::string &serialized);
  /// @brief Parse ArrayTreeT from file path using .tree format. Does not provide runtime meta-data.
  inline static ArrayTreeT fromPathNoRuntime(const std::string &path);

  /// @brief Is given node index a valid one?
  static constexpr bool isNodeIdValidValue(const NodeIdT &idx);

  /// @brief Is given node index valid for this tree?
  bool isNodeIdValid(const NodeIdT &idx) const;

  /// @brief Check that given node index is valid or throw.
  void checkNodeIdValidThrow(const NodeIdT &idx) const;

  /// @brief Translate given node index into a new one and returns the new index.
  NodeIdT translateNodeId(const NodeIdT &idx) const;

  /// @brief Get first node identifier for iteration.
  NodeIdT beginNodeId() const;
  /// @brief Get one behind last node identifier for iteration.
  NodeIdT endNodeId() const;

  /// @brief Get number of nodes currently in this tree.
  std::size_t nodeCount() const;
  /// @brief Are there any nodes in this tree?
  bool empty() const;

  /// @brief Clear all nodes from this tree and reset root.
  void clearNodes();

  /// @brief Get identifier of the root node.
  const NodeIdT &getRootId() const;
  /// @brief Is the root node of this tree valid?
  bool isRootNodeValid() const;

  /// @brief Get root node of this tree.
  NodeT &getRoot();
  /// @brief Get root node of this tree or nullptr if it is empty.
  const NodeT &getRoot() const;

  /// @brief Get node with given identifier or nullptr if it does not exist.
  NodeT &getNode(const NodeIdT &idx);
  /// @brief Get node with given identifier or nullptr if it does not exist.
  const NodeT &getNode(const NodeIdT &idx) const;

  /// @brief Get list of children for node with given identifier. Throws if node does not exist.
  const NodeChildArrayT &getNodeChildren(const NodeIdT &idx) const;

  /// @brief Get parent identifier for given node or throws if the node does not exist.
  const NodeIdT &getNodeParent(const NodeIdT &idx) const;

  /// @brief Create a new root node with given data and return its identifier.
  NodeIdT addRoot(const NodeDataT &data);

  /// @brief Create a new child node with given data and return its identifier.
  NodeIdT addNodeChild(const NodeIdT &idx, const NodeDataT &data);

  /// @brief Move provided child node to the new parent. Returns child index or INVALID_NODE_ID on error.
  const NodeIdT &addNodeChild(const NodeIdT &parentIdx, const NodeIdT &childIdx);

  /// @brief Remove given child node from provided parent. Returns child index or INVALID_NODE_ID on error.
  NodeIdT removeNodeChild(const NodeIdT &parentIdx, const NodeIdT &childIdx);

  /// @brief Set children for given parent index while fixing old and new children. Invalid child ids are skipped.
  void setNodeChildren(const NodeIdT &parentId, const std::vector<NodeIdT> &children);

  /// @brief Calculate bounding box for this tree.
  treeutil::BoundingBox getBoundingBox() const;
  /// @brief Swaps the values between dira and dirb (x - 1, y - 2, z - 3) Negative number means opposite direction.
  void swapCoords(int dira, int dirb);

  /// @brief Cleanup this ArrayTree and return its clean variant.
  ArrayTreeT cleanup() const;

  /// @brief Get path to file from which was this tree loaded from.
  const std::string &filePath() const;
  /// @brief Set path of file from which was the tree loaded from.
  void setFilePath(const std::string &filePath);

  /// @brief Has this tree been alredy loaded from file?
  bool loaded() const;
  /// @brief Set loaded state for this tree.
  void setLoaded(bool loaded);

  /// @brief Access the node ID translation map.
  const TranslationMapT &translationMap() const;
  /// @brief Access the node ID translation map.
  TranslationMapT &translationMap();

  /// @brief Access tree meta-data.
  MetaDataT &metaData();
  /// @brief Access tree meta-data.
  const MetaDataT &metaData() const;

  /// @brief Serialize the tree into a string.
  std::string serialize() const;

  /// @brief Save the tree into file from which it was loaded (filePath()).
  bool saveTree() const;
  /// @brief Save the tree into file specified by provided path.
  bool saveTree(const std::string &path) const;

  /// @brief Print debug information about tree nodes.
  void printNodeInfo() const;

  /// @brief Convert given node identifier to index into node array. Performs no checks!
  static std::size_t nodeIdToIdx(const NodeIdT &id);
  /// @brief Convert given node index into node array into node identifier. Performs no checks!
  static NodeIdT nodeIdxToId(std::size_t idx);

  /// Get forward begin iterator, root -> leaves, breadth-first.
  iterator begin();
  /// Get forward end iterator.
  iterator end();
  /// Get forward begin iterator, root -> leaves, breadth-first.
  const_iterator begin() const;
  /// Get forward end iterator.
  const_iterator end() const;

  /// Get reverse begin iterator, leaves -> root, depth-first.
  reverse_iterator rbegin();
  /// Get reverse end iterator.
  reverse_iterator rend();
  /// Get reverse begin iterator, leaves -> root, depth-first.
  const_reverse_iterator rbegin() const;
  /// Get reverse end iterator.
  const_reverse_iterator rend() const;

  /**
   * @brief Get begin iterator using requested style.
   *
   * @param style Style of iterator to use.
   * @param indirectNodes Access nodes in indirect manner? Useful when changing
   *  tree during the iteration process.
   * @param keepHistory Keep historical node information, including any user data.
   *
   * @return Returns the iterator.
   */
  iterator begin(IterationStyle style, bool indirectNodes = true, bool keepHistory = true);
  const_iterator begin(IterationStyle style, bool indirectNodes = true, bool keepHistory = true) const;

  /// @brief Get begin iterator using requested style. This version allows for user data.
  template <typename UserDataT>
  IteratorT<false, UserDataT> begin(IterationStyle style, bool indirectNodes = true, bool keepHistory = true);
  template <typename UserDataT>
  IteratorT<true, UserDataT> begin(IterationStyle style, bool indirectNodes = true, bool keepHistory = true) const;

  /**
   * @brief Get begin iterator using requested style.
   *
   * @param id Identifier of the node to start with.
   * @param style Style of iterator to use.
   * @param indirectNodes Access nodes in indirect manner? Useful when changing
   *  tree during the iteration process.
   * @param keepHistory Keep historical node information, including any user data.
   *
   * @return Returns the iterator.
   */
  iterator begin(const NodeIdT &id, IterationStyle style, bool indirectNodes = true, bool keepHistory = true);
  const_iterator begin(const NodeIdT &id, IterationStyle style, bool indirectNodes = true,
                       bool keepHistory = true) const;

  /// @brief Get begin iterator using requested style and starting node. This version allows for user data.
  template <typename UserDataT>
  IteratorT<false, UserDataT> begin(const NodeIdT &id, IterationStyle style, bool indirectNodes = true,
                                    bool keepHistory = true);
  template <typename UserDataT>
  IteratorT<true, UserDataT> begin(const NodeIdT &id, IterationStyle style, bool indirectNodes = true,
                                   bool keepHistory = true) const;

 private:
 protected:
  // Allow internal access between template instantiations.
  template <typename T1, typename T2>
  friend class ArrayTreeT;

  // TODO - Rewrite all recursive algorithms to use stack?

  /// @brief Procedure which serializes this tree to given stream using .tree format.
  template <typename ST>
  void saveTreeRecursion(ST &ss, const NodeIdT &currentId) const;

  /// @brief Create empty tree with a single root node, using given runtime meta-data.
  static ArrayTreeT emptyTree(const TreeRuntimeMetaData::Ptr &runtime);

  /// @brief De-serialize ArrayTree from given string.
  static ArrayTreeT parseTreeFromString(const std::string &serialized, const TreeRuntimeMetaData::Ptr &runtime);

  /// @brief De-serialize ArrayTree from .tree format.
  static ArrayTreeT parseTreeFromTreeString(const std::string &serialized, const TreeRuntimeMetaData::Ptr &runtime);

  /// @brief De-serialize ArrayTree from .json format.
  static ArrayTreeT parseTreeFromJSONString(const std::string &serialized, const TreeRuntimeMetaData::Ptr &runtime);

  /// Identifier of the root node.
  NodeIdT mRoot{INVALID_NODE_ID};
  /// List of nodes currently within this tree.
  NodeArrayT<NodeT> mNodes;
  /// Map used for translating old point IDs to new ones.
  TranslationMapT mTranslationMap;
  /// Path to file from which was this tree loaded from.
  std::string mFilePath{};
  /// Was this tree loaded?
  bool mLoaded{false};
  /// Meta-data for this tree.
  MetaDataT mMetaData;
};  // class ArrayTreeT

/// @brief Shortcut for the normally used ArrayTree.
using ArrayTree = ArrayTreeT<TreeNodeData, TreeMetaData>;

/// @brief Dummy used for passing ArrayTree past library boundaries.
class ArrayTreeDummy {
 public:
  /// @brief Copy from given ArrayTree.
  ArrayTreeDummy(const ArrayTree &tree) : mPtr{std::make_shared<ArrayTree>(tree)} {
  }

  /// @brief get pointer to the internal tree.
  const std::shared_ptr<ArrayTree> &ptr() const {
    return mPtr;
  }

 private:
  /// Pointer to the internal tree.
  std::shared_ptr<ArrayTree> mPtr{};

 protected:
};  // class ArrayTreeDummy

/// @brief Iterator usable for iteration over ArrayTree structure.
template <typename AT = ArrayTree, bool ConstIterator = false, typename UserDataT = void>
class TreeIteratorT {
 public:
  /// @brief Exception thrown when invalid operation is used on an iterator.
  struct InvalidIteratorException : public std::runtime_error {
    InvalidIteratorException(const char *msg) : std::runtime_error(msg) {
    }
  };  // struct InvalidIteratorException

  // Type shortcuts for the target ArrayTree type.
  /// @brief Type of this.
  using ThisT = TreeIteratorT<AT, ConstIterator, UserDataT>;
  /// @brief Type of this using other const-ness.
  using ThisOtherConstT = TreeIteratorT<AT, !ConstIterator>;
  /// @brief Target ArrayTree type.
  using ArrayTree = typename std::conditional<ConstIterator, const typename AT::ThisT, typename AT::ThisT>::type;
  /// @brief Operation style of the iterator.
  using Style = typename ArrayTree::IterationStyle;
  /// @brief Type used as node in the tree.
  using NodeT =
      typename std::conditional<ConstIterator, const typename ArrayTree::NodeT, typename ArrayTree::NodeT>::type;
  /// @brief Type used to index tree nodes.
  using NodeIdT = typename ArrayTree::NodeIdT;
  /// Value representing invalid index to a node.
  static constexpr auto INVALID_NODE_ID{ArrayTree::INVALID_NODE_ID};

  /// @brief Information structure for the current node.
  struct NodeInfoBase {
    /// Starting depth used for the initial nodes.
    static constexpr std::ptrdiff_t START_DEPTH{0u};

    /// Identifier of the node.
    NodeIdT identifier{INVALID_NODE_ID};
    /// Pointer to the node itself. May become invalid on operations with the tree.
    NodeT *node{nullptr};
    /// Depth of the node. May be positive (children) or negative (parents).
    std::ptrdiff_t depth{START_DEPTH};

    /// Identifier of the previous node.
    NodeIdT prevIdentifier{INVALID_NODE_ID};
    /// Pointer to the previous node. May become invalid on operations with the tree.
    NodeT *prevNode{nullptr};
  };  // Struct NodeinfoBase

  // Concrete NodeInfo definition:
  template <typename T>
  struct NodeInfoT;

  /// @brief Shortcut for the concrete NodeInfo type.
  using NodeInfo = NodeInfoT<UserDataT>;

  /// @brief Initialize end iterator.
  TreeIteratorT();
  /// @brief Initialize iterator for given tree.
  TreeIteratorT(ArrayTree &tree, Style iterationStyle = Style::BreadthFirstChildren, bool indirectNodes = true,
                bool keepHistory = true);

  /// @brief Initialize iterator from given node.
  TreeIteratorT(NodeT &node, ArrayTree &tree, Style iterationStyle = Style::BreadthFirstChildren,
                bool indirectNodes = true, bool keepHistory = true);
  /// @brief Initialize iterator from given node.
  TreeIteratorT(NodeIdT &node, ArrayTree &tree, Style iterationStyle = Style::BreadthFirstChildren,
                bool indirectNodes = true, bool keepHistory = true);

  /// @brief Clean up and destroy.
  ~TreeIteratorT();

  // Lazy copy operators:
  TreeIteratorT(const TreeIteratorT &other);
  TreeIteratorT &operator=(const TreeIteratorT &other);
  // Move operators:
  TreeIteratorT(TreeIteratorT &&other);
  TreeIteratorT &operator=(TreeIteratorT &&other);

  /// @brief Get operation style of this iterator.
  Style style() const;

  /// @brief Move to the next element, based on operation style. Pre-increment version.
  ThisT &operator++();
  /// @brief Move to the next element, based on operation style. Post-increment version.
  const ThisT operator++(int);

  /// @brief Move to the previous element, based on operation style. Pre-decrement version.
  ThisT &operator--();
  /// @brief Move to the previous element, based on operation style. Post-decrement version.
  const ThisT operator--(int);

  /// @brief Access current node.
  NodeT operator*();
  /// @brief Access current node.
  NodeT *operator->();

  /// @brief Access current node information.
  const NodeInfo &info() const;
  /// @brief Access current node information.
  NodeInfo &info();

  /// @brief Does this iterator currently point to a valid node?
  bool valid() const;

  // Comparison operators.
  bool operator==(const ThisT &other) const;
  bool operator!=(const ThisT &other) const;
  bool operator==(const ThisOtherConstT &other) const;
  bool operator!=(const ThisOtherConstT &other) const;

 private:
  /// @brief Check if we own the runtime data and make a copy if not.
  void checkRuntimeDataCopy();

  /// @brief Initialize from given tree - using root node or leaves.
  void initialize(ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory);
  /// @brief Initialize from given tree - using provided node.
  void initialize(NodeT &node, ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory);
  /// @brief Initialize from given tree - using provided node.
  void initialize(NodeIdT &node, ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory);

  /// @brief Record for a single queued node.
  struct NodeRecord {
    /// Node identifier.
    NodeIdT identifier{INVALID_NODE_ID};
    /// Previous node identifier.
    NodeIdT prevIdentifier{INVALID_NODE_ID};
  };  // struct NodeRecord

  /// @brief Runtime data used by the iterator.
  struct RuntimeData {
    /// Tree being iterated.
    ArrayTree *tree{nullptr};
    /// Queue used for storage of upcoming nodes.
    std::deque<NodeRecord> queue{};
    /// History of node information structures.
    std::unordered_map<NodeIdT, NodeInfo> history{};
    /// Identifier of the currently pointed to node.
    NodeIdT currentNode{INVALID_NODE_ID};
    /// Operation style of this iterator.
    Style style{Style::BreadthFirstChildren};
    /// Are we using indirect node access?
    bool indirectNodes{true};
    /// Should we keep all history available?
    bool keepHistory{true};

    /// Original owner of this data.
    ThisT *owner{nullptr};
  };  // struct RuntimeData

  /// Currently used runtime data.
  std::shared_ptr<RuntimeData> mRuntime{};

 protected:
};  // class TreeIteratorT

}  // namespace treeio

// Template implementation begin.

namespace treeio {

template <typename DataT>
TreeNodeT<DataT>::TreeNodeT() {
}

template <typename DataT>
TreeNodeT<DataT>::TreeNodeT(const DataT &data) : mNodeData{data} {
}

template <typename DataT>
template <typename OutDataT>
TreeNodeT<OutDataT> TreeNodeT<DataT>::copy() const {
  TreeNodeT<OutDataT> result{};

  // Copy all of the internal properties:
  result.mArrayParent = mArrayParent;
  result.mArrayId = mArrayId;
  result.mArrayChildren = mArrayChildren;
  // Explicitly cast the old data into the new data type.
  result.mNodeData = OutDataT(mNodeData);

  return result;
}

template <typename DataT>
const typename TreeNodeT<DataT>::NodeIdT &TreeNodeT<DataT>::parent() const {
  return mArrayParent;
}

template <typename DataT>
const typename TreeNodeT<DataT>::NodeIdT &TreeNodeT<DataT>::id() const {
  return mArrayId;
}

template <typename DataT>
const typename TreeNodeT<DataT>::NodeChildArrayT &TreeNodeT<DataT>::children() const {
  return mArrayChildren;
}

template <typename DataT>
typename TreeNodeT<DataT>::NodeDataT &TreeNodeT<DataT>::data() {
  return mNodeData;
}

template <typename DataT>
const typename TreeNodeT<DataT>::NodeDataT &TreeNodeT<DataT>::data() const {
  return mNodeData;
}

template <typename DataT>
typename TreeNodeT<DataT>::NodeChildArrayT &TreeNodeT<DataT>::children() {
  return mArrayChildren;
}

template <typename DataT, typename MetaDataT>
ArrayTreeT<DataT, MetaDataT>::ArrayTreeT() { /* Automatic */
}
template <typename DataT, typename MetaDataT>
ArrayTreeT<DataT, MetaDataT>::~ArrayTreeT() { /* Automatic */
}

template <typename DataT, typename MetaDataT>
template <typename OutDataT>
ArrayTreeT<OutDataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::copy() const {
  ArrayTreeT<OutDataT, MetaDataT> result{};

  // Copy simple properties:
  result.mRoot = mRoot;
  result.mTranslationMap = mTranslationMap;
  result.mFilePath = mFilePath;
  result.mLoaded = mLoaded;
  result.mMetaData = mMetaData;

  // Perform deep copy of the nodes:
  result.mNodes.resize(mNodes.size());
  for (std::size_t iii = 0u; iii < mNodes.size(); ++iii) {
    result.mNodes[iii] = mNodes[iii].template copy<OutDataT>();
  }

  return result;
}

template <typename DataT, typename MetaDataT>
template <typename RuntimeMetaDataT>
ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::fromPath(const std::string &path) {
  auto readTree{parseTreeFromString(treeutil::readWholeFile(path), treeutil::WrapperCtrT<RuntimeMetaDataT>())};
  readTree.mLoaded = true;
  readTree.mFilePath = path;

  return readTree;
}

template <typename DataT, typename MetaDataT>
template <typename RuntimeMetaDataT>
ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::fromString(const std::string &serialized) {
  return parseTreeFromString(serialized, treeutil::WrapperCtrT<RuntimeMetaDataT>());
}

template <typename DataT, typename MetaDataT>
inline ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::fromPathNoRuntime(const std::string &path) {
  auto readTree{parseTreeFromString(treeutil::readWholeFile(path), nullptr)};
  readTree.mLoaded = true;
  readTree.mFilePath = path;

  return readTree;
}

template <typename DataT, typename MetaDataT>
inline ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::fromStringNoRuntime(const std::string &serialized) {
  return parseTreeFromString(serialized, nullptr);
}

template <typename DataT, typename MetaDataT>
constexpr bool ArrayTreeT<DataT, MetaDataT>::isNodeIdValidValue(const NodeIdT &idx) {
  return idx != INVALID_NODE_ID;
}

template <typename DataT, typename MetaDataT>
bool ArrayTreeT<DataT, MetaDataT>::isNodeIdValid(const NodeIdT &idx) const {
  return isNodeIdValidValue(idx) && nodeIdToIdx(idx) < mNodes.size();
}

template <typename DataT, typename MetaDataT>
void ArrayTreeT<DataT, MetaDataT>::checkNodeIdValidThrow(const NodeIdT &idx) const {
  if (!isNodeIdValid(idx)) {
    throw InvalidNodeIdException("Invalid node identifier provided!");
  }
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeIdT ArrayTreeT<DataT, MetaDataT>::translateNodeId(const NodeIdT &idx) const {
  const auto findIt{mTranslationMap.find(idx)};
  if (findIt != mTranslationMap.end()) {
    return findIt->second;
  } else {
    return INVALID_NODE_ID;
  }
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeIdT ArrayTreeT<DataT, MetaDataT>::beginNodeId() const {
  return nodeIdxToId(0u);
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeIdT ArrayTreeT<DataT, MetaDataT>::endNodeId() const {
  return nodeIdxToId(nodeCount());
}

template <typename DataT, typename MetaDataT>
std::size_t ArrayTreeT<DataT, MetaDataT>::nodeCount() const {
  return mNodes.size();
}

template <typename DataT, typename MetaDataT>
bool ArrayTreeT<DataT, MetaDataT>::empty() const {
  return mNodes.empty();
}

template <typename DataT, typename MetaDataT>
void ArrayTreeT<DataT, MetaDataT>::clearNodes() {
  mNodes.clear();
  mRoot = INVALID_NODE_ID;
}

template <typename DataT, typename MetaDataT>
const typename ArrayTreeT<DataT, MetaDataT>::NodeIdT &ArrayTreeT<DataT, MetaDataT>::getRootId() const {
  return mRoot;
}

template <typename DataT, typename MetaDataT>
bool ArrayTreeT<DataT, MetaDataT>::isRootNodeValid() const {
  return isNodeIdValid(getRootId());
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeT &ArrayTreeT<DataT, MetaDataT>::getRoot() {
  return getNode(mRoot);
}
template <typename DataT, typename MetaDataT>
const typename ArrayTreeT<DataT, MetaDataT>::NodeT &ArrayTreeT<DataT, MetaDataT>::getRoot() const {
  return getNode(mRoot);
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeT &ArrayTreeT<DataT, MetaDataT>::getNode(const NodeIdT &idx) {
  checkNodeIdValidThrow(idx);
  return mNodes[nodeIdToIdx(idx)];
}
template <typename DataT, typename MetaDataT>
const typename ArrayTreeT<DataT, MetaDataT>::NodeT &ArrayTreeT<DataT, MetaDataT>::getNode(const NodeIdT &idx) const {
  checkNodeIdValidThrow(idx);
  return mNodes[nodeIdToIdx(idx)];
}

template <typename DataT, typename MetaDataT>
const typename ArrayTreeT<DataT, MetaDataT>::NodeChildArrayT &ArrayTreeT<DataT, MetaDataT>::getNodeChildren(
    const NodeIdT &idx) const {
  return getNode(idx).mArrayChildren;
}

template <typename DataT, typename MetaDataT>
const typename ArrayTreeT<DataT, MetaDataT>::NodeIdT &ArrayTreeT<DataT, MetaDataT>::getNodeParent(
    const NodeIdT &idx) const {
  return getNode(idx).mArrayParent;
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeIdT ArrayTreeT<DataT, MetaDataT>::addRoot(const NodeDataT &data) {
  const auto newId{nodeIdxToId(mNodes.size())};

  // Create the new node.
  NodeT newNode{data};
  newNode.mArrayId = newId;
  if (isNodeIdValid(mRoot)) {  // The old root is child of the new root.
    newNode.children().push_back(mRoot);
    getNode(mRoot).mArrayParent = newId;
  }
  mNodes.emplace_back(std::move(newNode));

  // Switch to new root.
  mRoot = newId;

  return newId;
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeIdT ArrayTreeT<DataT, MetaDataT>::addNodeChild(const NodeIdT &idx,
                                                                                          const NodeDataT &data) {
  if (!isNodeIdValid(idx)) {
    return INVALID_NODE_ID;
  }

  const auto newId{nodeIdxToId(mNodes.size())};

  // Create the new node.
  TreeNode newNode{data};
  newNode.mArrayId = newId;
  newNode.mArrayParent = idx;
  mNodes.emplace_back(std::move(newNode));

  // Add child node to the parent.
  getNode(idx).mArrayChildren.push_back(newId);

  return newId;
}

template <typename DataT, typename MetaDataT>
const typename ArrayTreeT<DataT, MetaDataT>::NodeIdT &ArrayTreeT<DataT, MetaDataT>::addNodeChild(
    const NodeIdT &parentIdx, const NodeIdT &childIdx) {
  if (!isNodeIdValid(parentIdx) || !isNodeIdValid(childIdx)) {
    return INVALID_NODE_ID;
  }

  auto &parentNode{getNode(parentIdx)};
  auto &childNode{getNode(childIdx)};

  const auto findIt{std::find(parentNode.mArrayChildren.begin(), parentNode.mArrayChildren.end(), childIdx)};
  if (findIt != parentNode.mArrayChildren.end()) {
    return childIdx;
  }

  // Add the child to the new parent.
  parentNode.mArrayChildren.push_back(childIdx);
  childNode.mArrayParent = parentIdx;

  return childIdx;
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeIdT ArrayTreeT<DataT, MetaDataT>::removeNodeChild(const NodeIdT &parentIdx,
                                                                                             const NodeIdT &childIdx) {
  auto &parentNode{getNode(parentIdx)};
  auto &childNode{getNode(childIdx)};

  const auto findIt{std::find(parentNode.mArrayChildren.begin(), parentNode.mArrayChildren.end(), childIdx)};
  if (findIt == parentNode.mArrayChildren.end()) {
    return childIdx;
  }

  // Remove the child from this parent.
  parentNode.mArrayChildren.erase(findIt);
  childNode.mArrayParent = INVALID_NODE_ID;

  return childIdx;
}

template <typename DataT, typename MetaDataT>
void ArrayTreeT<DataT, MetaDataT>::setNodeChildren(const NodeIdT &parentId, const std::vector<NodeIdT> &children) {
  if (!isNodeIdValid(parentId)) {
    return;
  }

  auto &parentNode{getNode(parentId)};

  // Clear current children.
  for (const auto &childId : parentNode.mArrayChildren) {
    if (isNodeIdValid(childId)) {
      getNode(childId).mArrayParent = INVALID_NODE_ID;
    }
  }

  // Set new children.
  parentNode.mArrayChildren.clear();
  parentNode.mArrayChildren.reserve(children.size());
  for (const auto &childId : children) {
    if (isNodeIdValid(childId)) {
      getNode(childId).mArrayParent = parentId;
      parentNode.mArrayChildren.push_back(childId);
    }
  }
}

template <typename DataT, typename MetaDataT>
treeutil::BoundingBox ArrayTreeT<DataT, MetaDataT>::getBoundingBox() const {
  auto minX{std::numeric_limits<float>::max()};
  auto minY{std::numeric_limits<float>::max()};
  auto minZ{std::numeric_limits<float>::max()};
  auto maxX{std::numeric_limits<float>::lowest()};
  auto maxY{std::numeric_limits<float>::lowest()};
  auto maxZ{std::numeric_limits<float>::lowest()};

  for (const auto &node : mNodes) {
    const auto &data{node.data()};
    minX = std::min(minX, data.pos.x);
    minY = std::min(minY, data.pos.y);
    minZ = std::min(minZ, data.pos.z);
    maxX = std::max(maxX, data.pos.x);
    maxY = std::max(maxY, data.pos.y);
    maxZ = std::max(maxZ, data.pos.z);
  }

  treeutil::BoundingBox bb{{minX, minY, minZ}, {maxX - minX, maxY - minY, maxZ - minZ}, true};
  return bb;
}

template <typename DataT, typename MetaDataT>
void ArrayTreeT<DataT, MetaDataT>::swapCoords(int dira, int dirb) {
  for (auto &node : mNodes) {
    node.data().swapNodeCoords(dira, dirb);
  }
}

template <typename DataT, typename MetaDataT>
ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::cleanup() const {
  if (mNodes.empty() || !isNodeIdValid(mRoot)) {
    return {};
  }

  // Copy base information.
  ArrayTree cleanTree{*this};
  cleanTree.mNodes.clear();

  // Clean up dead nodes:
  std::stack<NodeIdT> nodeStack{};
  std::map<NodeIdT, NodeIdT> nodeTranslation{};

  // Initialize with just the root node.
  nodeStack.push(mRoot);
  nodeTranslation.emplace(mRoot, nodeIdxToId(0u));
  cleanTree.mNodes.emplace_back(TreeNode{});

  while (!nodeStack.empty()) {  // Copy all accessible nodes.
    const auto currentNodeIdx{nodeStack.top()};
    nodeStack.pop();

    const auto findIt{nodeTranslation.find(currentNodeIdx)};
    const auto newNodeIdx{nodeIdToIdx(findIt->second)};
    const auto newNodeId{nodeIdxToId(newNodeIdx)};
    auto &newNode{cleanTree.getNode(newNodeId)};

    // We mark already processed nodes by setting arrayId.
    if (newNode.id()) {
      std::cout << "CleanupArrayTree: Found a loop, cutting it!" << std::endl;
      continue;
    }

    // Recover data for the current node.
    const auto &currentNode{getNode(currentNodeIdx)};

    // Copy the current node.
    newNode.mNodeData = currentNode.mNodeData;

    // Mark this node as processed.
    newNode.mArrayId = newNodeId;

    for (const auto &childIdx : currentNode.mArrayChildren) {  // Create all children of the current node.
      // Create the new node.
      const auto newChildIdx{cleanTree.mNodes.size()};
      const auto newChildId{nodeIdxToId(newChildIdx)};
      cleanTree.mNodes.emplace_back(TreeNode{});
      auto &newChildNode{cleanTree.mNodes.back()};

      // Fill some basic data.
      newChildNode.mArrayParent = newNodeId;

      // Register the new node.
      nodeTranslation.emplace(childIdx, newChildId);
      newNode.children().push_back(newChildId);

      // Add it for later processing.
      nodeStack.push(childIdx);
    }
  }

  cleanTree.mTranslationMap = std::move(nodeTranslation);

  return cleanTree;
}

template <typename DataT, typename MetaDataT>
const std::string &ArrayTreeT<DataT, MetaDataT>::filePath() const {
  return mFilePath;
}

template <typename DataT, typename MetaDataT>
void ArrayTreeT<DataT, MetaDataT>::setFilePath(const std::string &filePath) {
  mFilePath = filePath;
}

template <typename DataT, typename MetaDataT>
bool ArrayTreeT<DataT, MetaDataT>::loaded() const {
  return mLoaded;
}

template <typename DataT, typename MetaDataT>
void ArrayTreeT<DataT, MetaDataT>::setLoaded(bool loaded) {
  mLoaded = loaded;
}

template <typename DataT, typename MetaDataT>
const typename ArrayTreeT<DataT, MetaDataT>::TranslationMapT &ArrayTreeT<DataT, MetaDataT>::translationMap() const {
  return mTranslationMap;
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::TranslationMapT &ArrayTreeT<DataT, MetaDataT>::translationMap() {
  return mTranslationMap;
}

template <typename DataT, typename MetaDataT>
MetaDataT &ArrayTreeT<DataT, MetaDataT>::metaData() {
  return mMetaData;
}

template <typename DataT, typename MetaDataT>
const MetaDataT &ArrayTreeT<DataT, MetaDataT>::metaData() const {
  return mMetaData;
}

template <typename DataT, typename MetaDataT>
std::string ArrayTreeT<DataT, MetaDataT>::serialize() const {
  if (!isNodeIdValid(mRoot)) {
    return "";
  }

  std::stringstream ss{};

  // Output the metadata:
  ss << mMetaData.serialize() << "\n#####\n";

  // Traverse the tree and output it:
  saveTreeRecursion(ss, mRoot);

  return ss.str();
}

template <typename DataT, typename MetaDataT>
bool ArrayTreeT<DataT, MetaDataT>::saveTree() const {
  return saveTree(filePath());
}

template <typename DataT, typename MetaDataT>
bool ArrayTreeT<DataT, MetaDataT>::saveTree(const std::string &path) const {
  if (!isNodeIdValid(mRoot)) {
    return false;
  }

  // Open the output file.
  std::ofstream treeFile{};
  treeFile.open(path);
  if (!treeFile.is_open()) {
    return false;
  }

  treeFile << serialize();

  treeFile.close();

  return true;
}

template <typename DataT, typename MetaDataT>
void ArrayTreeT<DataT, MetaDataT>::printNodeInfo() const {
  std::cout << "Tree node information: " << std::endl;
  std::cout << "\tRoot node ID: " << mRoot << std::endl;
  for (auto nodeId = beginNodeId(); nodeId != endNodeId(); ++nodeId) {
    const auto nodeIdx{nodeIdToIdx(nodeId)};
    std::cout << "\tNode (" << nodeId << "), idx: " << nodeIdx << std::endl;

    const auto &node{mNodes[nodeIdx]};

    std::cout << "\t\tArray ID: " << node.id() << std::endl;
    std::cout << "\t\tParent ID: " << node.parent() << std::endl;

    std::cout << "\t\tChild IDs: ";
    for (const auto &childId : node.children()) {
      std::cout << childId << ", ";
    }
    std::cout << std::endl;

    std::cout << "\t\tPosition: " << node.data().pos.x << " " << node.data().pos.y << " " << node.data().pos.z
              << std::endl;
  }
  std::cout << "End of node info." << std::endl;
}

template <typename DataT, typename MetaDataT>
std::size_t ArrayTreeT<DataT, MetaDataT>::nodeIdToIdx(const NodeIdT &id) {
  return static_cast<std::size_t>(id) - 1u;
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::NodeIdT ArrayTreeT<DataT, MetaDataT>::nodeIdxToId(std::size_t idx) {
  return static_cast<NodeIdT>(idx + 1u);
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::iterator ArrayTreeT<DataT, MetaDataT>::begin() {
  return iterator(*this, IterationStyle::BreadthFirstChildren, true, true);
}
template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::iterator ArrayTreeT<DataT, MetaDataT>::end() {
  return iterator();
}
template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::const_iterator ArrayTreeT<DataT, MetaDataT>::begin() const {
  return const_iterator(*this, IterationStyle::BreadthFirstChildren, true, true);
}
template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::const_iterator ArrayTreeT<DataT, MetaDataT>::end() const {
  return const_iterator();
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::reverse_iterator ArrayTreeT<DataT, MetaDataT>::rbegin() {
  return reverse_iterator(*this, IterationStyle::DepthFirstParents, true, true);
}
template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::reverse_iterator ArrayTreeT<DataT, MetaDataT>::rend() {
  return reverse_iterator();
}
template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::const_reverse_iterator ArrayTreeT<DataT, MetaDataT>::rbegin() const {
  return const_reverse_iterator(*this, IterationStyle::DepthFirstParents, true, true);
}
template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::const_reverse_iterator ArrayTreeT<DataT, MetaDataT>::rend() const {
  return const_reverse_iterator();
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::iterator ArrayTreeT<DataT, MetaDataT>::begin(IterationStyle style,
                                                                                    bool indirectNodes,
                                                                                    bool keepHistory) {
  return iterator(*this, style, indirectNodes, keepHistory);
}
template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::const_iterator ArrayTreeT<DataT, MetaDataT>::begin(IterationStyle style,
                                                                                          bool indirectNodes,
                                                                                          bool keepHistory) const {
  return const_iterator(*this, style, indirectNodes, keepHistory);
}

template <typename DataT, typename MetaDataT>
template <typename UserDataT>
typename ArrayTreeT<DataT, MetaDataT>::template IteratorT<false, UserDataT> ArrayTreeT<DataT, MetaDataT>::begin(
    IterationStyle style, bool indirectNodes, bool keepHistory) {
  return IteratorT<false, UserDataT>(*this, style, indirectNodes, keepHistory);
}
template <typename DataT, typename MetaDataT>
template <typename UserDataT>
typename ArrayTreeT<DataT, MetaDataT>::template IteratorT<true, UserDataT> ArrayTreeT<DataT, MetaDataT>::begin(
    IterationStyle style, bool indirectNodes, bool keepHistory) const {
  return IteratorT<true, UserDataT>(*this, style, indirectNodes, keepHistory);
}

template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::iterator ArrayTreeT<DataT, MetaDataT>::begin(const NodeIdT &id,
                                                                                    IterationStyle style,
                                                                                    bool indirectNodes,
                                                                                    bool keepHistory) {
  return iterator(id, *this, style, indirectNodes, keepHistory);
}
template <typename DataT, typename MetaDataT>
typename ArrayTreeT<DataT, MetaDataT>::const_iterator ArrayTreeT<DataT, MetaDataT>::begin(const NodeIdT &id,
                                                                                          IterationStyle style,
                                                                                          bool indirectNodes,
                                                                                          bool keepHistory) const {
  return const_iterator(id, *this, style, indirectNodes, keepHistory);
}

template <typename DataT, typename MetaDataT>
template <typename UserDataT>
typename ArrayTreeT<DataT, MetaDataT>::template IteratorT<false, UserDataT> ArrayTreeT<DataT, MetaDataT>::begin(
    const NodeIdT &id, IterationStyle style, bool indirectNodes, bool keepHistory) {
  return IteratorT<false, UserDataT>(id, *this, style, indirectNodes, keepHistory);
}
template <typename DataT, typename MetaDataT>
template <typename UserDataT>
typename ArrayTreeT<DataT, MetaDataT>::template IteratorT<true, UserDataT> ArrayTreeT<DataT, MetaDataT>::begin(
    const NodeIdT &id, IterationStyle style, bool indirectNodes, bool keepHistory) const {
  return IteratorT<true, UserDataT>(id, *this, style, indirectNodes, keepHistory);
}

template <typename DataT, typename MetaDataT>
template <typename ST>
void ArrayTreeT<DataT, MetaDataT>::saveTreeRecursion(ST &ss, const NodeIdT &currentId) const {
  // Get data for current node.
  auto &currentNode{getNode(currentId)};
  const auto &currentData{currentNode.data()};

  // Serialize the node.
  ss << "(" << currentData.pos.x << "," << currentData.pos.y << "," << currentData.pos.z << "," << currentData.thickness
     << ")";

  // Move to the children.
  const auto &currentChildren{currentNode.children()};
  const auto multipleChildren{currentChildren.size() > 1u};

  // Serialize all children recursively.
  for (std::size_t iii = 0u; iii < currentChildren.size(); ++iii) {
    if (multipleChildren && iii != currentChildren.size() - 1u) {
      ss << "[";
    }
    saveTreeRecursion(ss, currentChildren[iii]);
    if (multipleChildren && iii != currentChildren.size() - 1u) {
      ss << "]";
    }
  }
}

template <typename DataT, typename MetaDataT>
ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::emptyTree(const TreeRuntimeMetaData::Ptr &runtime) {
  ArrayTree emptyTree{};

  NodeDataT rootNodeData{};
  emptyTree.addRoot(rootNodeData);

  MetaDataT metaData{};
  metaData.deserialize("", runtime);
  emptyTree.mMetaData = metaData;

  emptyTree.mLoaded = true;

  return emptyTree;
}

template <typename DataT, typename MetaDataT>
ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::parseTreeFromString(
    const std::string &serialized, const TreeRuntimeMetaData::Ptr &runtime) {
  // Detect the type of file:
  const auto dividerPosition{serialized.find_first_of("#####")};
  const auto firstCharacterPos{std::find_if(serialized.begin(), serialized.end(), [](auto c) {
    return !std::isspace(c);
  })};
  const auto firstCharacter{firstCharacterPos != serialized.end() ? *firstCharacterPos : '\0'};

  if (dividerPosition != std::string::npos || firstCharacter != '{') {
    try {
      return parseTreeFromTreeString(serialized, runtime);
    } catch (std::exception &e) {
      treeutil::Error << "Failed to parse tree from string! : \"" << e.what() << "\"" << std::endl;
      return emptyTree(runtime);
    }
  } else {
    try {
      return parseTreeFromJSONString(serialized, runtime);
    } catch (std::exception &e) {
      treeutil::Error << "Failed to parse tree from JSON! : \"" << e.what() << "\"" << std::endl;
      return emptyTree(runtime);
    }
  }
}

template <typename DataT, typename MetaDataT>
ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::parseTreeFromTreeString(
    const std::string &serialized, const TreeRuntimeMetaData::Ptr &runtime) {
  // Split meta info and nodes
  auto divider{serialized.find_first_of("#####")};
  auto dividernext{divider + 5u};
  if (divider == std::string::npos) {  // No divider found, assume there is no metadata and just read the branches:
    divider = 0u;
    dividernext = 0u;
  }
  const auto metatext{serialized.substr(0u, divider)};
  const auto nodetext{serialized.substr(dividernext)};

  // Parse it.
  MetaDataT metaData{};
  metaData.deserialize(metatext, runtime);

  // Parse the tree structure
  std::size_t ptr{0u};
  std::size_t depth{0u};
  const char brackets[4] = {'(', '[', ']'};
  const char closing[2] = {',', ')'};

  ArrayTree newTree;
  newTree.mLoaded = false;

  bool hasRoot = false;
  std::vector<NodeIdT> turtleDives{};  // for storing parent to go back to on each dive.
  NodeIdT current{INVALID_NODE_ID};
  // break skips to the end where the tree is assembled as if the reading went fine.
  while (depth >= 0u && ptr < nodetext.size()) {
    const auto nextb{nodetext.find_first_of(brackets, ptr)};
    if (nextb == std::string::npos) {  // No more brackets
      std::cerr << "npos";
      if (depth > 0u) {  // Mismatched parsing (some nodes were not closed)
        return newTree;
      } else {  // Ok end
        break;
      }
    }
    ptr = nextb + 1;  // move pointer
    const auto nbracket{nodetext[nextb]};
    const auto b_dive{nbracket == '['};
    const auto b_diveup{nbracket == ']'};
    const auto b_newnode{nbracket == '('};
    if (b_dive) {
      depth++;
      turtleDives.push_back(current);
    } else if (b_diveup) {
      if (depth == 0u) {  // Mismatched parsing (atemped to close square bracket at depth 0)
        newTree.mLoaded = false;
        return newTree;
      }
      depth--;
      // Make the last stored element the 'current' again and then delete it from the list
      current = turtleDives.back();
      turtleDives.pop_back();
    } else if (b_newnode) {
      NodeT myNode;
      // Parse params
      int nextsem;
      bool finishedNode = false;
      std::string subbie;
      // x
      nextsem = nodetext.find_first_of(closing, ptr);
      if (!finishedNode) {
        finishedNode |= nodetext[nextsem] == ')';
        subbie = nodetext.substr(ptr, nextsem - ptr);
        // std::cout << subbie << "\n";
        myNode.data().pos.x = std::stof(subbie);
        ptr = nextsem + 1;
      }
      // y
      nextsem = nodetext.find_first_of(closing, ptr);
      if (!finishedNode) {
        finishedNode |= nodetext[nextsem] == ')';
        subbie = nodetext.substr(ptr, nextsem - ptr);
        // std::cout << subbie << "\n";
        myNode.data().pos.y = std::stof(subbie);
        ptr = nextsem + 1;
      }
      // z
      nextsem = nodetext.find_first_of(closing, ptr);
      if (!finishedNode) {
        finishedNode |= nodetext[nextsem] == ')';
        subbie = nodetext.substr(ptr, nextsem - ptr);
        // std::cout << subbie << "\n";
        myNode.data().pos.z = std::stof(subbie);
        ptr = nextsem + 1;
      }
      // w
      nextsem = nodetext.find_first_of(closing, ptr);
      if (!finishedNode) {
        finishedNode |= nodetext[nextsem] == ')';
        subbie = nodetext.substr(ptr, nextsem - ptr);
        // std::cout << subbie << "\n";
        myNode.data().thickness = std::stof(subbie);
        ptr = nextsem + 1;
      }
      // insert my node at the end (since it's a struct) and then make it 'current':
      myNode.mArrayParent = current;
      NodeIdT myNodeId{INVALID_NODE_ID};
      if (hasRoot) {
        myNodeId = newTree.addNodeChild(current, myNode.data());
      } else {
        hasRoot = true;
        myNodeId = newTree.addRoot(myNode.data());
      }
      current = myNodeId;

      // move to the next closing bracket
      const auto nextb{nodetext.find_first_of(')', ptr - 1)};
      if (nextb == std::string::npos) {  // mismatched parsing (the current node was not closed)
        newTree.mLoaded = false;
        return newTree;
      }
      ptr = nextb + 1;
    }
  }
  // Assemble the final tree:
  newTree.mMetaData = metaData;
  newTree.mLoaded = true;

  return newTree;
}

namespace impl {

inline auto getRootJSONObject(const std::string &serialized) {
  const auto data{treeutil::containsOnlyWhiteSpaces(serialized) ? treeio::json{} : json::parse(serialized)};
  const auto isArray{data.type() == json::value_t::array};
  return isArray ? *data.begin() : data;
}

}  // namespace impl

template <typename DataT, typename MetaDataT>
ArrayTreeT<DataT, MetaDataT> ArrayTreeT<DataT, MetaDataT>::parseTreeFromJSONString(
    const std::string &serialized, const TreeRuntimeMetaData::Ptr &runtime) {
  // auto dataRoot{ impl::getRootJSONObject(serialized) };
  // auto data{ dataRoot };

  // for (auto &d : dataRoot)
  //{ if (d.contains("Count") && d.contains("Internodes")) { data = d; } }

  const json data = json::parse(serialized);

  if (!data.contains("Count") || !data.contains("Internodes")) {
    return {};
  }

  const auto nodeCount{data["Count"].get<std::size_t>()};
  const json internodes = data["Internodes"];

  ArrayTree tree{};
  tree.mLoaded = false;

  struct NodeInfo {
    /// Index of this node.
    std::size_t index{0u};
    /// Index of the parent node.
    std::size_t parent{0u};
    /// Gravelius ordering of this node.
    std::size_t gravelius{0u};
    /// Thickness of the segment.
    float thickness{0.0f};
    /// Level of this node.
    std::size_t level{0u};
    /// Age this node started existing at.
    std::size_t startAge{0u};
    /// Position of the node.
    Vector3D position{};
    /// List of child node indices.
    std::vector<std::size_t> children{};
  };  // struct NodeInfo

  const auto parseNode{[](const treeio::json &dat) {
    NodeInfo info{};

    // auto chosenDat{ dat };

    // for (auto &d : dat)
    //{ if (d.contains("Index") && d.contains("Parent")) { chosenDat = d; } }

    info.index = dat["Index"].get<std::size_t>();
    info.parent = dat["Parent"].get<std::size_t>();
    info.gravelius = dat["Gravelius Order"].get<std::size_t>();
    info.thickness = dat["Thickness"].get<float>();
    info.level = dat["Level"].get<std::size_t>();
    info.startAge = dat["Start Age"].get<std::size_t>();
    json pos = dat["Position"];
    // auto basePos{ chosenDat["Position"] };
    // auto pos{ basePos };
    // for (auto &d : basePos)
    //{ pos = d; }
    info.position = Vector3D{pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>()};
    info.children = dat["Children"].get<std::vector<std::size_t>>();

    return info;
  }};

  std::map<std::size_t, NodeInfo> nodes{};
  NodeInfo rootNode{};
  bool rootNodeFound{false};

  for (const auto node : internodes) {  // Parse all nodes within the file.
    const auto nodeInfo{parseNode(node)};
    if (!rootNodeFound) {
      rootNode = nodeInfo;
      rootNodeFound = true;
    }

    nodes.emplace(nodeInfo.index, nodeInfo);
  }

  struct NodeProcessingInfo {
    /// Information about the node.
    NodeInfo info{};
    /// Index of the parent node, already within the tree.
    NodeIdT parentId{INVALID_NODE_ID};
  };  // struct NodeProcessingInfo

  std::stack<NodeProcessingInfo> toProcess{};
  toProcess.push({rootNode, INVALID_NODE_ID});

  while (!toProcess.empty()) {  // Construct the tree.
    const auto node{toProcess.top()};
    toProcess.pop();
    NodeT newNode{};

    newNode.data().pos = node.info.position;
    newNode.data().thickness = node.info.thickness;

    const auto newNodeId{node.parentId == INVALID_NODE_ID ? tree.addRoot(newNode.data())
                                                          : tree.addNodeChild(node.parentId, newNode.data())};

    for (const auto &childId : node.info.children) {
      toProcess.push({nodes[childId], newNodeId});
    }
  }

  // Create meta-data and finalize the tree.
  MetaDataT metaData{};
  metaData.deserialize("", runtime);

  tree.mLoaded = true;
  tree.mMetaData = metaData;

  return tree;
}

#if 0

/// @brief Base case of NodeInfo without any user data.
template <typename AT, bool ConstIterator, typename UserDataT>
template <> struct TreeIteratorT<AT, ConstIterator, UserDataT>::NodeInfoT<void> : public NodeInfoBase { };
/// @brief NodeInfo with user data.
template <typename AT, bool ConstIterator, typename UserDataT>
template <typename T>
struct TreeIteratorT<AT, ConstIterator, UserDataT>::NodeInfoT : public NodeInfoBase
{
    /// User data container.
    T userData{ };
}; // struct NodeInfo

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>::TreeIteratorT()
{ /* Automatic */}

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>::TreeIteratorT(
    ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory)
{ initialize(tree, iterationStyle, indirectNodes, keepHistory); }

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>::TreeIteratorT(
    NodeT &node, ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory)
{ initialize(tree, iterationStyle, indirectNodes, keepHistory); }
template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>::TreeIteratorT(
    NodeIdT &node, ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory)
{ initialize(tree, iterationStyle, indirectNodes, keepHistory); }

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>::~TreeIteratorT()
{ /* Automatic */ }

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>::TreeIteratorT(const TreeIteratorT &other)
{ *this = other; }

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>&
    TreeIteratorT<AT, ConstIterator, UserDataT>::operator=(const TreeIteratorT &other)
{ mRuntime = other.mRuntime; mCurrentQueueOffset = other.mCurrentQueueOffset; return *this; }

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>::TreeIteratorT(TreeIteratorT &&other)
{ *this = std::move(other); }

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>&
    TreeIteratorT<AT, ConstIterator, UserDataT>::operator=(TreeIteratorT &&other)
{ mRuntime = other.mRuntime; mRuntime->owner = this; mCurrentQueueOffset = other.mCurrentQueueOffset; return *this; }

template <typename AT, bool ConstIterator, typename UserDataT>
typename TreeIteratorT<AT, ConstIterator, UserDataT>::Style
    TreeIteratorT<AT, ConstIterator, UserDataT>::style() const
{ return mRuntime->style; }

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>&
    TreeIteratorT<AT, ConstIterator, UserDataT>::operator++()
{
    if (!valid())
    { throw InvalidIteratorException("Unable to operator++ on invalid iterator!"); }

    checkRuntimeDataCopy();

    if (mRuntime->queue.empty())
    { // No more nodes to process -> Convert to end pointer.
        mRuntime->currentNodeId = INVALID_NODE_ID;
        return *this;
    }

    // Recover information about the next node in line and its predecessor.
    const auto nextNodeRecord{ mRuntime->queue.back() }; mRuntime->queue.pop_back();
    const auto nextNodeId{ nextNodeRecord.identifier };
    const auto prevNodeId{ nextNodeRecord.prevIdentifier };

    const auto findIt{ mRuntime->history.find(prevNodeId) };
    const auto *prevNodeInfo{
        findIt != mRuntime->history.end() ?
            &findIt->second :
            nullptr
    };

    // Add planned nodes to the queue.
    if (mRuntime->style == Style::DepthFirstChildren ||
        mRuntime->style == Style::BreadthFirstChildren)
    { // Parent to children -> Add all child nodes.
        const auto &nextNodeChildren{
            mRuntime->tree->getChildren(nextNodeId);
        };
        if (mRuntime->style == Style::DepthFirstChildren)
        { // Depth-first search.
            for (const auto &child : nextNodeChildren)
            { mQueue.push_back({ child, nextNodeId }); }
        }
        else // (mRuntime->style == Style::BreadthFirstChildren)
        { // Breadth-first search.
            for (const auto &child : nextNodeChildren)
            { mQueue.push_front({ child, nextNodeId }); }
        }
    }
    else // (mRuntime->style == Style::DepthFirstParents ||
         //  mRuntime->style == Style::BreadthFirstParents)
    {
        const auto &nextNodeParent{
            mRuntime->tree->getNode(nextNodeId).parent();
        };
        if (mRuntime->style == Style::DepthFirstParents)
        { // Depth-first search.
            mQueue.push_back({ nextNodeParent, nextNodeId });
        }
        else // (mRuntime->style == Style::BreadthFirstParents)
        { // Breadth-first search.
            mQueue.push_front({ nextNodeParent, nextNodeId });
        }
    }

    // Clear history if requested.
    if (!mRuntime->keepHistory)
    { mRuntime->history.clear(); }

    // Setup information for the current node.
    mRuntime->history.emplace(nextNodeId, NodeInfo{
        nextNodeId, &mRuntime->tree->getNode(nextNodeId),
        prevNodeInfo ? prevNodeInfo->depth : NodeInfo::START_DEPTH,
        prevNodeInfo ? prevNodeInfo->identifier : INVALID_NODE_ID,
        prevNodeInfo ? prevNodeInfo->node : nullptr,
    });

    // Move iterator to the next node.
    mRuntime->currentNodeId = nextNodeId;

    return *this;
}

template <typename AT, bool ConstIterator, typename UserDataT>
const TreeIteratorT<AT, ConstIterator, UserDataT>
    TreeIteratorT<AT, ConstIterator, UserDataT>::operator++(int)
{ ThisT copy{ *this }; operator++(); return copy; };

template <typename AT, bool ConstIterator, typename UserDataT>
TreeIteratorT<AT, ConstIterator, UserDataT>&
    TreeIteratorT<AT, ConstIterator, UserDataT>::operator--()
{
    if (!valid())
    { throw InvalidIteratorException("Unable to operator-- on invalid iterator!"); }

    if (!mRuntime->keepHistory)
    { throw InvalidIteratorException("Unable to operator-- without keepHistory == true!"); }

    const auto currentNodeInfo{ info() };

    checkRuntimeDataCopy();

    if (currentNodeInfo.prevIdentifier == INVALID_NODE_ID)
    { // No more nodes to process -> Convert to end pointer.
        mRuntime->currentNodeId = INVALID_NODE_ID;
        return *this;
    }

    // Move to the previous node.
    mRuntime->currentNodeId = currentNodeinfo.prevIdentifier;
}

template <typename AT, bool ConstIterator, typename UserDataT>
const TreeIteratorT<AT, ConstIterator, UserDataT>
    TreeIteratorT<AT, ConstIterator, UserDataT>::operator--(int)
{ ThisT copy{ *this }; operator--(); return copy; };

template <typename AT, bool ConstIterator, typename UserDataT>
typename TreeIteratorT<AT, ConstIterator, UserDataT>::NodeT
    TreeIteratorT<AT, ConstIterator, UserDataT>::operator*()
{ return *info().node; }

template <typename AT, bool ConstIterator, typename UserDataT>
typename TreeIteratorT<AT, ConstIterator, UserDataT>::NodeT*
    TreeIteratorT<AT, ConstIterator, UserDataT>::operator->()
{ return info().node; }

template <typename AT, bool ConstIterator, typename UserDataT>
const typename TreeIteratorT<AT, ConstIterator, UserDataT>::NodeInfo&
    TreeIteratorT<AT, ConstIterator, UserDataT>::info() const
{ return const_cast<ThisT*>(this)->info(); }

template <typename AT, bool ConstIterator, typename UserDataT>
typename TreeIteratorT<AT, ConstIterator, UserDataT>::NodeInfo&
    TreeIteratorT<AT, ConstIterator, UserDataT>::info()
{
    if (!valid())
    { throw InvalidIteratorException("Cannot operator* on an invalid iterator!"); }

    return mRuntime->history[mRuntime->currentNode];
}

template <typename AT, bool ConstIterator, typename UserDataT>
bool TreeIteratorT<AT, ConstIterator, UserDataT>::valid() const
{ return mRuntime && mRuntime->currentNode != INVALID_NODE_ID; }

template <typename AT, bool ConstIterator, typename UserDataT>
bool TreeIteratorT<AT, ConstIterator, UserDataT>::operator==(
    const ThisT &other) const
{
    return
        // Compare iterators sharing runtime data.
        (mRuntime == other.mRuntime) ||
        // Compare explicit end iterator and end iterator.
        (mRuntime && mRuntime->currentNode == INVALID_NODE_ID && !other.mRuntime)
        // Compare iterators pointing at the same node.
        (mRuntime && other.mRuntime && mRuntime->currentNode == other.mRuntime->currentNode &&
            mRuntime->tree == other.mRuntime->tree)
}

template <typename AT, bool ConstIterator, typename UserDataT>
bool TreeIteratorT<AT, ConstIterator, UserDataT>::operator!=(
    const ThisT &other) const
{ return !(*this == other); }

template <typename AT, bool ConstIterator, typename UserDataT>
bool TreeIteratorT<AT, ConstIterator, UserDataT>::operator==(
    const ThisOtherConstT &other) const
{
    return
        // Compare iterators sharing runtime data.
        (mRuntime == other.mRuntime) ||
        // Compare explicit end iterator and end iterator.
        (mRuntime && mRuntime->currentNode == INVALID_NODE_ID && !other.mRuntime)
            // Compare iterators pointing at the same node.
            (mRuntime && other.mRuntime && mRuntime->currentNode == other.mRuntime->currentNode &&
             mRuntime->tree == other.mRuntime->tree)
}

template <typename AT, bool ConstIterator, typename UserDataT>
bool TreeIteratorT<AT, ConstIterator, UserDataT>::operator!=(
    const ThisOtherConstT &other) const
{ return !(*this == other); }

template <typename AT, bool ConstIterator, typename UserDataT>
void TreeIteratorT<AT, ConstIterator, UserDataT>::checkRuntimeDataCopy()
{
    if (!mRuntime)
    { // No runtime data exists -> Initialize.
        mRuntime = std::make_shared<RuntimeData>();
        mRuntime->owner = this;
    }
    else if (mRuntime->owner != this)
    { // We are using runtime data from other instance -> Make a copy.
        mRuntime = std::make_shared<RuntimeData>(*mRuntime);
        mRuntime->owner = this;
    }
}

template <typename AT, bool ConstIterator, typename UserDataT>
void TreeIteratorT<AT, ConstIterator, UserDataT>::initialize(
    ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory)
{
    // Initialize runtime:
    checkRuntimeDataCopy();
    mRuntime->tree = &tree;
    mRuntime->queue = { };
    mRuntime->history = { };
    mRuntime->currentNode = INVALID_NODE_ID;
    mRuntime->style = iterationStyle;
    mRuntime->indirectNodes = indirectNodes;
    mRuntime->keepHistory = keepHistory;

    // Initialize nodes:
    switch (mRuntime->style)
    {
        case Style::DepthFirstChildren:
        case Style::BreadthFirstChildren:
        { // Going parents -> children, use root.
            if (tree.isRootNodeValid())
            {
                mRuntime->queue.emplace_back(NodeInfo{
                    tree.getRootId(), &tree.getRootNode(), NodeInfo::START_DEPTH,
                    INVALID_NODE_ID, nullptr
                });
            }
            break;
        }
        case Style::DepthFirstParents:
        case Style::BreadthFirstParents:
        { // Going children -> parents, use leaves
            for (auto nodeId = tree.beginNodeId(); nodeId != tree.endNodeId(); ++nodeId)
            { // Search for all leaves in the input tree.
                const auto &currentNode{ tree.getNode(nodeId) };
                if (currentNode.children.size() == 0)
                { // Found a leaf -> Add it.
                    mRuntime->queue.emplace_back(NodeInfo{
                        nodeId, &currentNode, NodeInfo::START_DEPTH,
                        INVALID_NODE_ID, nullptr
                    });
                }
            }
            break;
        }
    }
}

template <typename AT, bool ConstIterator, typename UserDataT>
void TreeIteratorT<AT, ConstIterator, UserDataT>::initialize(
    NodeT &node, ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory)
{ initialize(node.id(), tree, iterationStyle, indirectNodes, keepHistory); }

template <typename AT, bool ConstIterator, typename UserDataT>
void TreeIteratorT<AT, ConstIterator, UserDataT>::initialize(
    NodeIdT &node, ArrayTree &tree, Style iterationStyle, bool indirectNodes, bool keepHistory)
{
    // Initialize runtime:
    checkRuntimeDataCopy();
    mRuntime->tree = &tree;
    mRuntime->queue = { };
    mRuntime->history = { };
    mRuntime->currentNode = INVALID_NODE_ID;
    mRuntime->style = iterationStyle;
    mRuntime->indirectNodes = indirectNodes;
    mRuntime->keepHistory = keepHistory;

    // Initialize the single node:
    mRuntime->queue.emplace_back(NodeInfo{
        node, &tree.getNode(node), NodeInfo::START_DEPTH,
        INVALID_NODE_ID, nullptr
    });
}

#endif

}  // namespace treeio

// Template implementation end.

#endif  // TREEIO_TREE
