#pragma once
#include "ForestDescriptor.hpp"
#include "SorghumField.hpp"
#include "SorghumPointCloudScanner.hpp"
#include "SorghumState.hpp"
#include "SorghumStateGenerator.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreePointCloudScanner.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class DatasetGenerator {
 public:
  static void GenerateTreeTrunkMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
                                    int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                    const std::string& treeMeshOutputPath, const std::string& treeTrunkOutputPath,
                                    const std::string& treeInfoPath);

  static void GenerateTreeMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
                               int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings,
                               const std::string& treeMeshOutputPath);

  static void GenerateTreeMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
                               std::vector<int> targetTreeNodeCount,
                               const TreeMeshGeneratorSettings& meshGeneratorSettings,
                               const std::string& treeMeshOutputPath);

  static void GeneratePointCloudForTree(const TreePointCloudPointSettings& pointSettings,
                                        const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
                                        const std::string& treeParametersPath, float deltaTime, int maxIterations,
                                        int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                        const std::string& pointCloudOutputPath, bool exportTreeMesh,
                                        const std::string& treeMeshOutputPath);
  static void GeneratePointCloudForForest(int gridSize, float gridDistance, float randomShift,
                                          const TreePointCloudPointSettings& pointSettings,
                                          const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
                                          const std::string& treeParametersFolderPath, float deltaTime,
                                          int maxIterations, int maxTreeNodeCount,
                                          const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                          const std::string& pointCloudOutputPath);
  static void GeneratePointCloudForForestPatch(const glm::ivec2& gridSize,
                                               const TreePointCloudPointSettings& pointSettings,
                                               const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
                                               const std::shared_ptr<ForestPatch>& forestPatch,
                                               const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                               const std::string& pointCloudOutputPath);
  static void GeneratePointCloudForForestPatchJoinedSpecies(
      const glm::ivec2& gridSize, const TreePointCloudPointSettings& pointSettings,
      const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
      const std::shared_ptr<ForestPatch>& forestPatch, const std::string& speciesFolderPath,
      const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::string& pointCloudOutputPath);
  static void GeneratePointCloudForSorghumPatch(const SorghumFieldPatch& pattern,
                                                const std::shared_ptr<SorghumStateGenerator>& sorghumDescriptor,
                                                const SorghumPointCloudPointSettings& pointSettings,
                                                const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
                                                const SorghumMeshGeneratorSettings& sorghumMeshGeneratorSettings,
                                                const std::string& pointCloudOutputPath);
};

}  // namespace eco_sys_lab