/**
 * @author Tomas Polasek, David Hrusa
 * @date 5.7.2020
 * @version 1.0
 * @brief Tree chain generation utilities.
 */

#include "TreeIOChains.hpp"

namespace treeutil {

TreeChains::InternalNodeData::InternalNodeData() { /* Automatic */
}
TreeChains::InternalNodeData::InternalNodeData(const treeio::TreeNodeData &original) : InternalNodeData() {
  pos = original.pos;
  thickness = original.thickness;
  calculatedThickness = original.thickness;
  freezeThickness = original.freezeThickness;

  if (std::isnan(thickness) || std::isinf(thickness)) {
    thickness = 0.0f;
    calculatedThickness = 0.0f;
    freezeThickness = false;
  }
}

TreeChains::TreeChains() { /* Automatic */
}
TreeChains::TreeChains(const treeio::ArrayTree &tree) : TreeChains() {
  buildTree(tree);
}
TreeChains::~TreeChains() { /* Automatic */
}

bool TreeChains::buildTree(const treeio::ArrayTree &tree) {
  // Create copy of the provided tree, including its structure and base properties.
  mInternalTree = tree.copy<InternalNodeData>();
  mLeafNodes.clear();
  mChains.clear();
  mLeafChains.clear();

  if (!generateUpwardPassInformation(mInternalTree)) {
    std::cout << "Failed to generate upward-pass information!" << std::endl;
    return false;
  }
  if (!generateDownwardPassInformation(mInternalTree)) {
    std::cout << "Failed to generate downward-pass information!" << std::endl;
    return false;
  }
  if (!generateOrthoBases(mInternalTree)) {
    std::cout << "Failed to generate orthonormal bases!" << std::endl;
    return false;
  }
  if (!generateNodeChains(mInternalTree, mChains, mLeafChains, mMaxChainDepth, mMaxChainGraveliusDepth)) {
    std::cout << "Failed to generate node chains!" << std::endl;
    return false;
  }

  return true;
}

void TreeChains::cascadeUpwards(UpwardPropFunT fun) {
  // Starting the algorithm from root towards the leaves:
  auto prematureStop{false};
  std::stack<NodeIdT> vertexStack{};
  vertexStack.emplace(mInternalTree.getRootId());

  while (!vertexStack.empty() && !prematureStop) {  // Process all vertices root to leaves.
    const auto currentNode{vertexStack.top()};
    vertexStack.pop();
    // Perform provided operation.
    prematureStop = fun(mInternalTree, currentNode);
    // Add child vertices.
    for (const auto &cId : mInternalTree.getNodeChildren(currentNode)) {
      vertexStack.emplace(cId);
    }
  }
}

void TreeChains::cascadeDownwards(DownwardPropFunT fun) {
  // Starting the algorithm from leaves towards the root:
  auto prematureStop{false};
  std::set<NodeIdT> currentNodes{mLeafNodes.begin(), mLeafNodes.end()};
  std::set<NodeIdT> newNodes{};
  std::set<NodeIdT> finishedNodes{};

  while (!currentNodes.empty() &&
         !prematureStop) {  // Proceed in waves from the leaf nodes while storing parent vertices to the newVertices.
    for (const auto &currentNode : currentNodes) {  // Process nodes in current wave.
      // Stop loops and repeats.
      if (finishedNodes.find(currentNode) != finishedNodes.end()) {
        continue;
      }

      // Check all children are ready.
      auto childrenReady{true};
      for (const auto &cId : mInternalTree.getNodeChildren(currentNode)) {
        if (finishedNodes.find(cId) == finishedNodes.end()) {
          childrenReady = false;
        }
      }

      // Skip parents with non-ready children, keeping them for later processing.
      if (!childrenReady) {
        continue;
      }

      // Perform requested operation.
      prematureStop = fun(mInternalTree, currentNode);

      // Schedule parent for processing.
      if (mInternalTree.isNodeIdValid(mInternalTree.getNodeParent(currentNode))) {
        newNodes.emplace(mInternalTree.getNodeParent(currentNode));
      }

      // Mark this node as finished.
      finishedNodes.emplace(currentNode);
    }
    // Switch to the accumulated parent vertices and continue with the algorithm.
    currentNodes.swap(newNodes);
    newNodes.clear();
  }
}

const TreeChains::InternalArrayTree &TreeChains::internalTree() const {
  return mInternalTree;
}

const TreeChains::NodeIdStorage &TreeChains::leafNodes() const {
  return mLeafNodes;
}

const TreeChains::ChainStorage &TreeChains::chains() const {
  return mChains;
}

const TreeChains::ChainIdxStorage &TreeChains::leafChains() const {
  return mLeafChains;
}

std::size_t TreeChains::maxChainDepth() const {
  return mMaxChainDepth;
}

std::size_t TreeChains::maxChainGraveliusDepth() const {
  return mMaxChainGraveliusDepth;
}

TreeChains::CompactChainStorage TreeChains::generateCompactChains(float maxLength) const {
  const auto &allChains{mChains};
  CompactChainStorage compactChains{};

  if (allChains.empty()) {
    return compactChains;
  }

  /// @brief Helper for compact chain creation.
  struct ChainRecord {
    /// Index of the source chain in allChains.
    NodeChain::ChainIdxT srcChainIdx{NodeChain::INVALID_CHAIN_IDX};

    /// Parent compact chain index.
    CompactNodeChain::ChainIdxT parentChainIdx{CompactNodeChain::INVALID_CHAIN_IDX};

    /// Index used by this compacted chain within the parents compactedChains array.
    std::size_t parentCompactedChainIdx{CompactNodeChain::INVALID_CHAIN_IDX};
  };  // struct ChainRecord

  /// @brief Helper for compact child chains.
  struct ChildChainRecord {
    /// Index of the source child chain in allChains.
    NodeChain::ChainIdxT srcChildIdx{NodeChain::INVALID_CHAIN_IDX};

    /// Index used by this compacted chain within the parents compactedChains array.
    std::size_t parentCompactedChainIdx{CompactNodeChain::INVALID_CHAIN_IDX};

    /// Length accumulated up to this child chain.
    float accumulatedLength{0.0f};
  };  // struct ChildChainRecord

  // Initialize processing with the root chain.
  std::stack<ChainRecord> toProcess{};
  toProcess.push({0u});

  while (!toProcess.empty()) {  // Process all input chains.
    const auto chainRecord{toProcess.top()};
    toProcess.pop();
    const auto &srcChain{allChains[chainRecord.srcChainIdx]};

    // Create the compact chain.
    const auto compactChainIdx{compactChains.size()};
    compactChains.push_back({});
    auto &compactChain{compactChains.back()};

    compactChain.compactedChains.push_back({CompactNodeChain::INVALID_CHAIN_IDX, chainRecord.srcChainIdx});
    compactChain.parentChain = {0u, chainRecord.parentChainIdx};
    if (compactChain.parentChain.chainIdx != NodeChain::INVALID_CHAIN_IDX) {
      compactChains[compactChain.parentChain.chainIdx].childChains.push_back(
          {chainRecord.parentCompactedChainIdx, compactChainIdx});
    }

    std::stack<ChildChainRecord> childChains{};
    for (const auto &ccIdx : srcChain.childChains) {
      childChains.push({ccIdx, 0u, 0.0f});
    }

    while (!childChains.empty()) {
      const auto childChainRecord{childChains.top()};
      childChains.pop();
      const auto childChainIdx{childChainRecord.srcChildIdx};
      const auto &srcChildChain{allChains[childChainIdx]};
      const auto chainLength{childChainRecord.accumulatedLength + srcChildChain.calculateChainLength(mInternalTree)};

      if (chainLength <= maxLength &&
          !srcChildChain.childChains
               .empty()) {  // Compact child chain into this one, only in case when it is not a leaf.
        const auto parentCompactedChainsIdx{compactChain.compactedChains.size()};
        for (const auto &ccIdx : srcChildChain.childChains) {
          childChains.push({ccIdx, parentCompactedChainsIdx, chainLength});
        }
        compactChain.compactedChains.push_back({parentCompactedChainsIdx, childChainIdx});
      } else {  // Child chain is longer, keep it for later processing.
        toProcess.push({childChainIdx, compactChainIdx, childChainRecord.parentCompactedChainIdx});
      }
    }
  }

  return compactChains;
}

std::size_t TreeChains::removeChainsDownToDepth(std::size_t depth) {
  std::size_t totalChainsMarked{0u};

  for (std::size_t iii = 0u; iii < mChains.size();
       ++iii) {  // Find chains at exactly the depth and recursively remove all following.
    if (mChains[iii].chainDepth == depth) {
      totalChainsMarked += markChainForRemoval(mInternalTree, mChains, iii);
    }
  }

  return totalChainsMarked;
}

std::size_t TreeChains::removeLeafChains(std::size_t count) {
  std::size_t totalChainsMarked{0u};

  std::set<NodeChain::ChainIdxT> chainsToRemove{};
  for (const auto &leafChainIdx : mLeafChains) {  // Accumulate chains to be removed.
    auto chainToRemove{leafChainIdx};
    for (std::size_t iii = 0u; iii < count - 1u; ++iii) {  // Descent given number of chains.
      chainToRemove = mChains[chainToRemove].parentChain;
    }

    chainsToRemove.emplace(chainToRemove);
  }

  // Remove all of the accumulated chains.
  for (const auto &chainIdx :
       chainsToRemove) {  // Find chains at exactly the depth and recursively remove all following.
    totalChainsMarked += markChainForRemoval(mInternalTree, mChains, chainIdx);
  }

  return totalChainsMarked;
}

std::size_t TreeChains::removeLeafChainsGravelius(std::size_t order) {
  std::size_t totalChainsMarked{0u};

  std::set<NodeChain::ChainIdxT> chainsToRemove{};
  for (const auto &leafChainIdx : mLeafChains) {  // Accumulate chains to be removed.
    auto chainToRemove{leafChainIdx};
    while (chainToRemove != NodeChain::INVALID_CHAIN_IDX &&
           mChains[chainToRemove].graveliusOrder < order) {  // Descent given number of chains.
      chainToRemove = mChains[chainToRemove].parentChain;
    }

    if (chainToRemove != NodeChain::INVALID_CHAIN_IDX && mChains[chainToRemove].graveliusOrder <= order) {
      chainsToRemove.emplace(chainToRemove);
    }
  }

  // Remove all of the accumulated chains.
  for (const auto &chainIdx :
       chainsToRemove) {  // Find chains at exactly the depth and recursively remove all following.
    totalChainsMarked += markChainForRemoval(mInternalTree, mChains, chainIdx);
  }

  return totalChainsMarked;
}

std::size_t TreeChains::remoChainsDownToGraveliusDepth(std::size_t depth) {
  std::size_t totalChainsMarked{0u};

  for (std::size_t iii = 0u; iii < mChains.size();
       ++iii) {  // Find chains at exactly the depth and recursively remove all following.
    if (mChains[iii].graveliusDepth == depth) {
      totalChainsMarked += markChainForRemoval(mInternalTree, mChains, iii);
    }
  }

  return totalChainsMarked;
}

bool TreeChains::applyChangesTo(treeio::ArrayTree &tree) const {
  // Start with the root node.
  std::stack<treeio::ArrayTree::NodeIdT> vertices{};
  vertices.emplace(tree.getRootId());

  while (!vertices.empty()) {  // Apply changes recursively going through the tree.
    const auto currentId{vertices.top()};
    vertices.pop();
    auto &currentNode{tree.getNode(currentId)};
    const auto &internalNode{mInternalTree.getNode(currentId)};

    // TODO - Apply changes to the node data?

    std::vector<treeio::ArrayTree::NodeIdT> resultChildArray{};
    for (const auto &cid : tree.getNodeChildren(currentId)) {
      const auto &internalChildNode{mInternalTree.getNode(cid)};
      if (!internalChildNode.data().markedForRemoval) {
        vertices.push(cid);
        resultChildArray.push_back(cid);
      }
    }
    tree.setNodeChildren(currentId, resultChildArray);
  }

  return true;
}

bool TreeChains::generateUpwardPassInformation(InternalArrayTree &tree) {
  /// @brief Helper structure for computing the depth.
  struct DepthHelper {
    /// The vertex itself.
    NodeIdT vertex{};
    /// Parent vertex.
    NodeIdT parent{};
    /// Depth from root.
    std::size_t depth{};
    /// Accumulated distance from root.
    float distance{};
  };  // struct DepthHelper

  std::stack<DepthHelper> vertexStack{};
  vertexStack.emplace(DepthHelper{tree.getRootId(), INVALID_NODE_ID, 0u, 0.0f});

  while (!vertexStack.empty()) {  // Process all vertices root to leaves.
    const auto currentHelper{vertexStack.top()};
    vertexStack.pop();
    // Update depth and distance of the current vertex.
    auto &currentNode{tree.getNode(currentHelper.vertex)};
    currentNode.data().depth = currentHelper.depth;
    currentNode.data().distance = currentHelper.distance;

    for (const auto &cId : tree.getNodeChildren(currentHelper.vertex)) {  // Add child vertices.
      const auto distanceFromParent{(tree.getNode(cId).data().pos - currentNode.data().pos).length()};
      vertexStack.emplace(DepthHelper{cId, currentHelper.vertex, currentHelper.depth + 1u,
                                      currentHelper.distance + distanceFromParent});
    }

    if (tree.getNodeChildren(currentHelper.vertex).empty()) {  // We found a leaf node.
      mLeafNodes.push_back(currentHelper.vertex);
    }
  }

  return true;
}

bool TreeChains::generateDownwardPassInformation(InternalArrayTree &tree) {
  // Initialize the set with leaf nodes:
  std::set<NodeIdT> currentVertices{mLeafNodes.begin(), mLeafNodes.end()};
  std::set<NodeIdT> newVertices{};
  std::set<NodeIdT> finishedVertices{};

  while (!currentVertices
              .empty()) {  // Proceed in waves from the leaf nodes while storing parent vertices to the newVertices.
    for (const auto &currentVertex : currentVertices) {  // Calculate child count of this vertex.
      auto &currentNode{tree.getNode(currentVertex).data()};
      if (finishedVertices.find(currentVertex) != finishedVertices.end()) {
        continue;
      }

      std::size_t childCount{0u};
      float childLength{0.0f};
      std::size_t graveliusMaxChildren{0u};
      std::size_t graveliusMaxChildOrder{1u};
      auto foundUnfinished{false};
      const auto parent{tree.getNodeParent(currentVertex)};

      for (const auto &cId : tree.getNodeChildren(
               currentVertex)) {  // Accumulate child counts for all child vertices and store parent for next iteration.
        if (finishedVertices.find(cId) == finishedVertices.end()) {
          foundUnfinished = true;
          break;
        }
        const auto &childNode{tree.getNode(cId).data()};

        childCount += childNode.totalChildCount;

        const auto currentToChildLength{(childNode.pos - currentNode.pos).length()};
        childLength += childNode.totalChildLength + currentToChildLength;

        if (childNode.graveliusOrder > graveliusMaxChildOrder) {
          graveliusMaxChildOrder = childNode.graveliusOrder;
          graveliusMaxChildren = 0u;
        }
        if (childNode.graveliusOrder == graveliusMaxChildOrder) {
          graveliusMaxChildren++;
        }
      }

      // Skip this node until we have all required children calculated.
      if (foundUnfinished) {
        continue;
      }

      // We have all required children -> Continue with the parent node.
      if (parent != INVALID_NODE_ID) {
        newVertices.emplace(parent);
      }

      // Add one for the current vertex.
      childCount += 1u;
      // Store for later use:
      currentNode.totalChildCount = childCount;
      currentNode.totalChildLength = childLength;

      if (graveliusMaxChildren > 1u) {  // We found a tributary joining.
        currentNode.graveliusOrder = graveliusMaxChildOrder + 1u;
      } else {  // Continue with the same stream.
        currentNode.graveliusOrder = graveliusMaxChildOrder;
      }

      finishedVertices.emplace(currentVertex);
    }
    // Switch to the accumulated parent vertices and continue with the algorithm.
    currentVertices.swap(newVertices);
    newVertices.clear();
  }

  return true;
}

bool TreeChains::generateOrthoBases(InternalArrayTree &tree) {
  /// @brief Helper structure for computing the orthonormal bases.
  struct BasisHelper {
    /// The vertex itself.
    NodeIdT vertex{};
    /// Parent vertex.
    NodeIdT parent{};
  };  // struct BasisHelper

  // Calculate the initial basis for the root node.
  const auto rootBasis{calculateBasisFromChildren(tree, tree.getRootId())};
  tree.getNode(tree.getRootId()).data().basis = rootBasis;

  // Start the algorithm with the root node.
  std::stack<BasisHelper> vertexStack{};
  vertexStack.emplace(BasisHelper{tree.getRootId(), INVALID_NODE_ID});

  while (!vertexStack.empty()) {  // Process all vertices root to leaves.
    const auto currentHelper{vertexStack.top()};
    vertexStack.pop();

    // Skip the root node, which has already been initialized.
    if (currentHelper.parent != INVALID_NODE_ID) {  // Processing non-root node -> Minimize rotation frames.
      const auto &parentNode{tree.getNode(currentHelper.parent)};
      auto &currentNode{tree.getNode(currentHelper.vertex)};

      if (Vector3D::distance(currentNode.data().pos, parentNode.data().pos) <
          MINIMUM_MRF_DISTANCE) {  // The two nodes are nearly identical -> use same information as the parent does.
        // Save the MRF as the new basis.
        currentNode.data().basis = parentNode.data().basis;
      } else {  // The Two nodes are different -> calculate next step.
        // Recover parent frame, which is already correctly rotated.
        const FrenetFrame parentFrenetFrame{parentNode.data().pos, parentNode.data().basis.bitangent,
                                            // parentNode.data().basis.tangent
                                            parentNode.data().basis.direction};

        // Calculate tangent and build starting frame for the current node.
        const auto currentBasis{calculateBasisFromParent(tree, currentHelper.vertex)};
        const FrenetFrame currentFrenetFrame{currentNode.data().pos, currentBasis.bitangent,
                                             // currentBasis.tangent
                                             currentBasis.direction};

        // Perform minimal rotation correction.
        const auto minimalRotationFrame{doubleReflectionRMF(parentFrenetFrame, currentFrenetFrame)};

        // Save the MRF as the new basis.
        currentNode.data().basis = OrthoBasis{
            currentBasis.direction,
            // minimalRotationFrame.tan,
            Vector3D::crossProduct(minimalRotationFrame.rot, minimalRotationFrame.tan), minimalRotationFrame.rot};
      }
    }

    // Move to the child vertices.
    for (const auto &cId : tree.getNodeChildren(currentHelper.vertex)) {
      vertexStack.emplace(BasisHelper{cId, currentHelper.vertex});
    }
  }

  return true;
}

bool TreeChains::generateNodeChains(const InternalArrayTree &tree, ChainStorage &chains, ChainIdxStorage &leafChains,
                                    std::size_t &maxChainDepth, std::size_t &maxGraveliusDepth) {
  /// @brief Helper structure for computing the chains.
  struct ChainHelper {
    /// The vertex itself.
    NodeIdT vertex{};
    /// Parent vertex.
    NodeIdT parent{INVALID_NODE_ID};
    /// Index of the current chain.
    std::size_t currentChain{NodeChain::INVALID_CHAIN_IDX};
    /// Gravelius depth.
    std::size_t graveliusDepth{0u};
  };  // struct ChainHelper

  // Initialize the process from the root node.
  std::stack<ChainHelper> vertexStack{};
  vertexStack.emplace(ChainHelper{tree.getRootId(), INVALID_NODE_ID, 0u, 0u});
  chains.push_back(NodeChain{});
  maxChainDepth = std::numeric_limits<std::size_t>::min();

  while (!vertexStack.empty()) {  // Process the whole tree from the root towards the leaves.
    const auto currentHelper{vertexStack.top()};
    vertexStack.pop();
    // We will be adding chains on the way, not safe to keep reference to it...
    const auto currentChainIdx{currentHelper.currentChain};

    // Copy Gravelius order of the highest order - from the root of this chain.
    chains[currentChainIdx].graveliusOrder = tree.getNode(currentHelper.vertex).data().graveliusOrder;
    // Save computed gravelius depth.
    chains[currentChainIdx].graveliusDepth = currentHelper.graveliusDepth;
    maxGraveliusDepth = std::max<std::size_t>(maxGraveliusDepth, currentHelper.graveliusDepth);

    // Find the first branching point or leaf and add all nodes in order.
    auto foundEnd{false};
    auto itVertex{currentHelper.vertex};
    auto itParent{currentHelper.parent};

    if (itParent != INVALID_NODE_ID) {
      chains[currentChainIdx].nodes.push_back(itParent);
    }

    while (!foundEnd) {  // Search for the chain end.
      chains[currentChainIdx].nodes.push_back(itVertex);

      const auto &children{tree.getNodeChildren(itVertex)};
      const auto childCount{children.size()};
      if (childCount == 1u || (childCount > 1u && itVertex == tree.getRootId())) {  // Found the next link in the chain.
        itParent = itVertex;
        itVertex = children[0];
      } else {  // Found end-point or branching point.
        foundEnd = true;
      }
    }
    // CurrentChain.nodes now contains all of the nodes.
    const auto finalVertex{itVertex};
    const auto finalParent{itParent};

    // Generate the adjacent chains:
    auto vertexChildren{tree.getNodeChildren(finalVertex)};
    if (currentHelper.vertex == tree.getRootId() &&
        currentHelper.vertex != finalVertex) {  // Add root node children, if there are any - roots for example.
      const auto rootChildren{tree.getNodeChildren(currentHelper.vertex)};
      vertexChildren.insert(vertexChildren.end(), rootChildren.begin() + 1u, rootChildren.end());
    }
    for (const auto &cid : vertexChildren) {  // Add new child-chain and register it.
      const auto childChainIdx{chains.size()};

      NodeChain nodeChain{};
      nodeChain.parentChain = currentHelper.currentChain;
      nodeChain.chainDepth = chains[currentChainIdx].chainDepth + 1u;
      maxChainDepth = std::max<std::size_t>(maxChainDepth, nodeChain.chainDepth);
      const auto childGraveliusOrder{tree.getNode(cid).data().graveliusOrder};
      const auto childGraveliusDepth{childGraveliusOrder < chains[currentChainIdx].graveliusOrder
                                         ? chains[currentChainIdx].graveliusDepth
                                         : chains[currentChainIdx].graveliusDepth + 1u};

      chains.push_back(nodeChain);
      vertexStack.emplace(ChainHelper{cid, finalVertex, childChainIdx, childGraveliusDepth});
      chains[currentChainIdx].childChains.push_back(childChainIdx);
    }

    if (tree.getNodeChildren(finalVertex).empty()) {  // Found a leaf chain.
      leafChains.push_back(currentChainIdx);
    }
  }

  return true;
}

TreeChains::FrenetFrame TreeChains::doubleReflectionRMF(const FrenetFrame &rotatedFrame,
                                                        const FrenetFrame &inputFrame) const {
  /*
   * Using the original variable names from the "Computation of Rotation Minimizing Frames":
   *   i -> 1
   *   i + 1 -> 2
   */

  const auto &x1{rotatedFrame.pos};
  const auto &r1{rotatedFrame.rot};
  const auto &t1{rotatedFrame.tan};
  const auto &x2{inputFrame.pos};
  const auto &r2{inputFrame.rot};
  const auto &t2{inputFrame.tan};

  // Compute reflection vector between the two centers.
  const auto v1{x2 - x1};
  const auto c1{Vector3D::dotProduct(v1, v1) + std::numeric_limits<float>::epsilon()};
  // Perform reflection of the r1 and t1 about the reflection vector v1.
  const auto r1l{r1 - ((2.0f / c1) * Vector3D::dotProduct(v1, r1) * v1)};
  const auto t1l{t1 - ((2.0f / c1) * Vector3D::dotProduct(v1, t1) * v1)};

  // Compute reflection vector between the two tangents of the input frame.
  const auto v2{t2 - t1l};
  const auto c2{Vector3D::dotProduct(v2, v2) + std::numeric_limits<float>::epsilon()};

  // Rotate the rotation vector for the input frame, oriented by the rotatedFrame.
  const auto r2c{r1l - ((2.0f / c2) * Vector3D::dotProduct(v2, r1l) * v2)};

  // Build the resulting frame from calculated values:
  FrenetFrame resultFrame{};
  resultFrame.pos = x2;
  resultFrame.rot = r2c;
  resultFrame.tan = t2;

  return resultFrame;
}

TreeChains::OrthoBasis TreeChains::calculateBasisFromChildren(const InternalArrayTree &tree,
                                                              const NodeIdT &nodeId) const {
  Vector3D childPositionSum{};
  const auto childCount{tree.getNodeChildren(nodeId).size()};
  for (const auto &cid : tree.getNodeChildren(nodeId)) {
    childPositionSum += tree.getNode(cid).data().pos;
  }

  const auto parentPosition{tree.getNode(nodeId).data().pos};
  const auto childPosition{childCount > 0u ?
                                           // Fallback to up vector when no children are present.
                               (parentPosition + Vector3D{0.0f, 1.0f, 0.0f})
                                           :
                                           // Use average child position when multiple children are present.
                               (childPositionSum / static_cast<float>(childCount))};

  return calculateBasis(parentPosition, childPosition);
};

TreeChains::OrthoBasis TreeChains::calculateBasisFromParent(const InternalArrayTree &tree,
                                                            const NodeIdT &nodeId) const {
  const auto parentId{tree.getNodeParent(nodeId)};

  // Fallback to basis from children if there is no parent for this node.
  if (parentId == INVALID_NODE_ID) {
    return calculateBasisFromChildren(tree, nodeId);
  };

  const auto parentPosition{tree.getNode(parentId).data().pos};
  const auto childPosition{tree.getNode(nodeId).data().pos};

  return calculateBasis(parentPosition, childPosition);
}

TreeChains::OrthoBasis TreeChains::calculateBasis(const Vector3D &srcPos, const Vector3D &dstPos) const {
  // Calculate orthonormal basis:
  const auto direction{(dstPos - srcPos).normalized()};
  const auto startTangent{Vector3D{1.0f, 0.0f, 0.0f}.dot(direction) >= (0.999f) ? Vector3D{0.0f, 1.0f, 0.0f}
                                                                                : Vector3D{1.0f, 0.0f, 0.0f}};
  const auto tangent{(startTangent - startTangent.dot(direction) * direction).normalized()};
  const auto bitangent{direction.crossProduct(tangent).normalized()};

  // Build the basis:
  OrthoBasis result{};
  result.direction = direction;
  result.tangent = tangent;
  result.bitangent = bitangent;

  return result;
}

std::size_t TreeChains::markChainForRemoval(InternalArrayTree &tree, ChainStorage &chains,
                                            const NodeChain::ChainIdxT &chainIdx) const {
  // Initialize the process with provided chain.
  std::stack<NodeChain::ChainIdxT> chainsToRemove{};
  chainsToRemove.emplace(chainIdx);

  std::size_t markedchains{0u};

  while (!chainsToRemove.empty()) {  // Repeat for all chains recursively.
    const auto currentChainIdx{chainsToRemove.top()};
    chainsToRemove.pop();
    auto &chain{chains[currentChainIdx]};

    // Mark for removal
    chain.markedForRemoval = true;
    markedchains++;
    for (const auto &nid : chain.nodes) {
      tree.getNode(nid).data().markedForRemoval = true;
    }

    // Continue with child chains.
    for (const auto &childChainIdx : chain.childChains) {
      chainsToRemove.emplace(childChainIdx);
    }
  }

  return markedchains;
}

}  // namespace treeutil
