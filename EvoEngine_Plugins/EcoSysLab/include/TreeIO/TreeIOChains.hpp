/**
 * @author Tomas Polasek, David Hrusa
 * @date 5.7.2020
 * @version 1.0
 * @brief Tree chain generation utilities.
 */

#ifndef TREEIO_CHAIN_H
#define TREEIO_CHAIN_H

#include "TreeIOTree.hpp"
#include "TreeIOUtils.hpp"

namespace treeutil {

/// @brief Helper class used for generating chain ArrayTree representation.
class TreeChains : public treeutil::PointerWrapper<TreeChains> {
 public:
  /// @brief Orthographic basis for a branch.
  struct OrthoBasis {
    /// Main direction of the branch.
    Vector3D direction{};
    /// Direction tangent to the main direction.
    Vector3D tangent{};
    /// Direction tangent to both main direction and the tangent.
    Vector3D bitangent{};
  };  // struct OrthoBasis

  /// @brief Chain representing un-branching sequence of nodes.
  struct NodeChain {
    /// @brief Type used for chain indexing.
    using ChainIdxT = std::size_t;

    /// Value used for invalid chain indices.
    static constexpr auto INVALID_CHAIN_IDX{std::numeric_limits<ChainIdxT>::max()};

    /// @brief Lambda function operating on segments.
    template <typename TreeT, typename ReturnT,
              typename NodeT = typename std::conditional_t<std::is_const_v<TreeT>, const typename TreeT::NodeT,
                                                           typename TreeT::NodeT>>
    using SegmentFun =
        std::conditional_t<std::is_same_v<ReturnT, void>, std::function<ReturnT(NodeT &, NodeT &)>,
                           std::function<ReturnT(NodeT &, NodeT &, const other_than_void_t<ReturnT, int> &)>>;

    /// @brief Lambda function operating on nodes.
    template <typename TreeT, typename ReturnT,
              typename NodeT = typename std::conditional_t<std::is_const_v<TreeT>, const typename TreeT::NodeT,
                                                           typename TreeT::NodeT>>
    using NodeFun = std::conditional_t<std::is_same_v<ReturnT, void>, std::function<ReturnT(NodeT &)>,
                                       std::function<ReturnT(NodeT &, const other_than_void_t<ReturnT, int> &)>>;

    /// @brief Run given lambda on each segment in this chain.
    template <typename TreeT>
    void forEachSegment(TreeT &tree, const SegmentFun<TreeT, void> &lambda) const;

    /// @brief Run given lambda on each segment in this chain.
    template <typename TreeT, typename ReturnT>
    ReturnT forEachSegment(TreeT &tree, const SegmentFun<TreeT, ReturnT> &lambda, const ReturnT &initial = {}) const;

    /// @brief Run given lambda on each node in this chain.
    template <typename TreeT>
    void forEachNode(TreeT &tree, const NodeFun<TreeT, void> &lambda) const;

    /// @brief Run given lambda on each node in this chain.
    template <typename TreeT, typename ReturnT>
    ReturnT forEachNode(TreeT &tree, const NodeFun<TreeT, ReturnT> &lambda, const ReturnT &initial = {}) const;

    /// @brief Calculate chain length using nodes in given tree.
    template <typename TreeT>
    auto calculateChainLength(const TreeT &tree) const;

    /**
     * List of node indices contained within this chain in order from lowest depth to highest.
     * First and last nodes are shared with parent and child chains respectively.
     */
    std::vector<treeio::ArrayTree::NodeIdT> nodes{};
    /// Index of the parent chain - Closer to the root.
    ChainIdxT parentChain{INVALID_CHAIN_IDX};
    /// List of child chains indices - Towards the leaves.
    std::vector<ChainIdxT> childChains{};
    /// Depth of the chain in number of chains from the root chain.
    std::size_t chainDepth{};
    /// Gravelius order of this chain.
    std::size_t graveliusOrder{};
    /// Depth based on gravelius order.
    std::size_t graveliusDepth{};

    /// Is this chain marked for removal?
    bool markedForRemoval{false};
  };  // struct NodeChain

  /// Description of chains, possibly containing multiple sub-chains.
  struct CompactNodeChain {
    /// @brief Type used for chain indexing.
    using ChainIdxT = NodeChain::ChainIdxT;

    /// Value used for invalid chain indices.
    static constexpr auto INVALID_CHAIN_IDX{NodeChain::INVALID_CHAIN_IDX};

    /// @brief Combination of compact chain index and index to its originator in compactedChains array.
    struct CompactChainOrigin {
      /// Index within the compactedChains array.
      std::size_t originIdx{INVALID_CHAIN_IDX};
      /// Compacted chain index.
      ChainIdxT chainIdx{INVALID_CHAIN_IDX};
    };  // struct CompactChain

    /// Index of the parent compact chain - Closer to the root.
    CompactChainOrigin parentChain{};
    /// List of child compact chains indices - Towards the leaves.
    std::vector<CompactChainOrigin> childChains{};
    /// List of chain indices compacted within this one.
    std::vector<CompactChainOrigin> compactedChains{};
  };  // struct CompactNodeChain

  /// @brief Container for node data used by the internal tree.
  struct InternalNodeData {
    /// @brief Default initialization;
    InternalNodeData();
    /// @brief Copy common information from the original tree node.
    InternalNodeData(const treeio::TreeNodeData &original);

    /// Position of this node in model-space.
    treeutil::Vector3D pos{};
    /// Branch thickness at this node.
    float thickness{};
    /// Calculated branch thickness at this node.
    float calculatedThickness{};
    /// Freeze thickness to the pre-set value?
    bool freezeThickness{false};

    /// Did we recalculate radius of this branch?
    bool recalculatedRadius{false};
    /// Depth of vertex from the root vertex.
    std::size_t depth{};
    /// Distance of vertex from the root along the edges.
    float distance{};
    /// Orthographic basis for each graph vertex.
    OrthoBasis basis{};
    /// Total number of child vertices, including all sub-trees.
    std::size_t totalChildCount{};
    /// Total length of child edges, including all sub-trees.
    float totalChildLength{};
    /// Gravelius order of this node.
    std::size_t graveliusOrder{};
    /// Index of the chain containing this vertex.
    NodeChain::ChainIdxT chainIndex{NodeChain::INVALID_CHAIN_IDX};

    /// Is this node marked for removal?
    bool markedForRemoval{false};
  };  // struct InternalNodeData

  /// Tree type used by this class.
  using InternalArrayTree = treeio::ArrayTreeT<InternalNodeData, treeio::TreeMetaData>;
  /// Type used to refer to nodes.
  using NodeIdT = InternalArrayTree::NodeIdT;
  /// Value used to specify invalid node ID.
  static constexpr auto INVALID_NODE_ID{InternalArrayTree::INVALID_NODE_ID};
  /// Type used to store a list of node ids.
  using NodeIdStorage = std::vector<NodeIdT>;
  /// Type used to store the chains.
  using ChainStorage = std::vector<NodeChain>;
  /// Type used to store the compact chains.
  using CompactChainStorage = std::vector<CompactNodeChain>;
  /// Type used to store a list of chain indices.
  using ChainIdxStorage = std::vector<NodeChain::ChainIdxT>;

  /// @brief Function used for upward propagation tasks - current tree and current node ID are provided.
  using UpwardPropFunT = std::function<bool(InternalArrayTree &, const NodeIdT &)>;
  /// @brief Function used for downward propagation tasks - current tree and current node ID are provided.
  using DownwardPropFunT = std::function<bool(InternalArrayTree &, const NodeIdT &)>;

  // Constructors:

  /// @brief Initialize helper structures.
  TreeChains();
  /// @brief Initialize helper structures and generate tree chains.
  TreeChains(const treeio::ArrayTree &tree);
  /// @brief Cleanup and destroy.
  ~TreeChains();

  // Allow copy and move:
  TreeChains(const TreeChains &other) = default;
  TreeChains(TreeChains &&other) = default;
  TreeChains &operator=(const TreeChains &other) = default;
  TreeChains &operator=(TreeChains &&other) = default;

  // Builders:

  /// @brief Generate tree chains and additional structures for given tree.
  bool buildTree(const treeio::ArrayTree &tree);

  // Oprations:

  /// @brief Perform computation by upwards propagation - from root towards the leaves. Returned bool used for premature
  /// stopping.
  void cascadeUpwards(UpwardPropFunT fun);
  /// @brief Perform computation by downwards propagation - from leaves towards the root. Returned bool used for
  /// premature stopping.
  void cascadeDownwards(DownwardPropFunT fun);

  // Accessors:

  /// @brief Access the internal tree and its data.
  const InternalArrayTree &internalTree() const;
  /// @brief Access the list of leaf nodes in the internalTree().
  const NodeIdStorage &leafNodes() const;

  /// @brief Access the chains. The first chain is always the root one.
  const ChainStorage &chains() const;
  /// @brief Access the list of leaf chains in the chains().
  const ChainIdxStorage &leafChains() const;
  /// @brief Get maximum depth of all of the chains.
  std::size_t maxChainDepth() const;
  /// @brief Get maximum Gravelius depth of all of the chains.
  std::size_t maxChainGraveliusDepth() const;
  /// @brief Generate list of compacted chains with chains <= maxLength merged with their parents.
  CompactChainStorage generateCompactChains(float maxLength) const;

  // Manipulators:

  /// @brief Mark all chains and corresponding nodes down to and including given depth for removal. Returns chains
  /// marked.
  std::size_t removeChainsDownToDepth(std::size_t depth);
  /// @brief Mark all chains up to given distance from leaves for removal. Returns chains marked.
  std::size_t removeLeafChains(std::size_t count);
  /// @brief Mark all chains up to given Gravelius order for removal. Returns chains marked.
  std::size_t removeLeafChainsGravelius(std::size_t order);
  /// @brief Mark all chains and corresponding nodes down to and including given Gravelius depth for removal. Returns
  /// chains marked.
  std::size_t remoChainsDownToGraveliusDepth(std::size_t depth);

  // Appliers:

  /// @brief Apply all internal changes to given tree.
  bool applyChangesTo(treeio::ArrayTree &tree) const;

 private:
  /// @brief Minimum distance of two nodes in order to compute the MRF.
  static constexpr auto MINIMUM_MRF_DISTANCE{0.01f};

  /// @brief Frenet frame used for rotation frame minimization.
  struct FrenetFrame {
    /// Position of the point.
    Vector3D pos{};
    /// Rotation vector.
    Vector3D rot{};
    /// Tangent vector.
    Vector3D tan{};
  };  // stuct FrenetFrame

  /// @brief Generate information going from root to the leaves of the current tree.
  bool generateUpwardPassInformation(InternalArrayTree &tree);
  /// @brief Generate information going from leaves to the root of the current tree.
  bool generateDownwardPassInformation(InternalArrayTree &tree);
  /// @brief Generate orthonormal bases for the current tree.
  bool generateOrthoBases(InternalArrayTree &tree);
  /// @brief Generate node chains for the current tree.
  bool generateNodeChains(const InternalArrayTree &tree, ChainStorage &chains, ChainIdxStorage &leafChains,
                          std::size_t &maxChainDepth, std::size_t &maxGraveliusDepth);

  /**
   * @brief Perform double reflection rotation minimization for given frames.
   * @param rotatedFrame Frame rotated to the correct orientation for the parent node.
   * @param inputFrame Input frame for the child node, which should be oriented as the rotatedFrame.
   * @return Returns frame rotated as the rotatedFrame created from the inputFrame.
   */
  FrenetFrame doubleReflectionRMF(const FrenetFrame &rotatedFrame, const FrenetFrame &inputFrame) const;

  /// @brief Calculate basis for given node ID using only its children.
  OrthoBasis calculateBasisFromChildren(const InternalArrayTree &tree, const NodeIdT &nodeId) const;

  /// @brief Calculate basis for given node ID using only its parent.
  OrthoBasis calculateBasisFromParent(const InternalArrayTree &tree, const NodeIdT &nodeId) const;

  /// @brief Calculate basis for node at srcPos, which continues with dstPos.
  OrthoBasis calculateBasis(const Vector3D &srcPos, const Vector3D &dstPos) const;

  /// @brief Mark given chain, all of its children and associated nodes for removal. Returns number of chains marked.
  std::size_t markChainForRemoval(InternalArrayTree &tree, ChainStorage &chains,
                                  const NodeChain::ChainIdxT &chainIdx) const;

  /// Internal tree used by this instance.
  InternalArrayTree mInternalTree{};
  /// List leaf node ids from mInternalTree.
  NodeIdStorage mLeafNodes{};
  /// List of chains making up the whole tree. First chain is the root one.
  ChainStorage mChains{};
  /// List of indices of leaf chains from mChains.
  ChainIdxStorage mLeafChains{};
  /// Maximum depth of all of the chains.
  std::size_t mMaxChainDepth{};
  /// Maximum Gravelius depth of all of the chains.
  std::size_t mMaxChainGraveliusDepth{};

 protected:
};  // class TreeChains

}  // namespace treeutil

// Template implementation begin.

namespace treeutil {

template <typename TreeT>
void TreeChains::NodeChain::forEachSegment(TreeT &tree, const SegmentFun<TreeT, void> &lambda) const {
  if (nodes.size() < 2u) {
    return;
  }

  auto lastNodeIdx{nodes[0u]};
  for (auto currentNodeIt = ++nodes.begin(); currentNodeIt != nodes.end(); ++currentNodeIt) {
    const auto currentNodeIdx{*currentNodeIt};
    lambda(tree.getNode(lastNodeIdx), tree.getNode(currentNodeIdx));
  }
}

template <typename TreeT, typename ReturnT>
ReturnT TreeChains::NodeChain::forEachSegment(TreeT &tree, const SegmentFun<TreeT, ReturnT> &lambda,
                                              const ReturnT &initial) const {
  if (nodes.size() < 2u) {
    return ReturnT{};
  }

  ReturnT returnVal{initial};

  auto lastNodeIdx{nodes[0u]};
  for (auto currentNodeIt = ++nodes.begin(); currentNodeIt != nodes.end(); ++currentNodeIt) {
    const auto currentNodeIdx{*currentNodeIt};
    returnVal = lambda(tree.getNode(lastNodeIdx), tree.getNode(currentNodeIdx), returnVal);
  }

  return returnVal;
}

template <typename TreeT>
void TreeChains::NodeChain::forEachNode(TreeT &tree, const NodeFun<TreeT, void> &lambda) const {
  for (const auto &currentNodeIdx : nodes) {
    lambda(tree.getNode(currentNodeIdx));
  }
}

template <typename TreeT, typename ReturnT>
ReturnT TreeChains::NodeChain::forEachNode(TreeT &tree, const NodeFun<TreeT, ReturnT> &lambda,
                                           const ReturnT &initial) const {
  ReturnT returnVal{initial};

  for (const auto &currentNodeIdx : nodes) {
    returnVal = lambda(tree.getNode(currentNodeIdx, returnVal));
  }

  return returnVal;
}

template <typename TreeT>
auto TreeChains::NodeChain::calculateChainLength(const TreeT &tree) const {
  return forEachSegment(
      tree,
      [](auto &node1, auto &node2, auto &runningLength) {
        const auto segmentLength{node1.data().pos.distanceTo(node2.data().pos)};
        return runningLength + segmentLength;
      },
      0.0f);
}

}  // namespace treeutil

// Template implementation end.

#endif  // TREEIO_CHAIN_H
