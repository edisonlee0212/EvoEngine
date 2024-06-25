//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#ifdef BUILD_WITH_RAYTRACER
#  include <RayTracerLayer.hpp>
#endif
#include "BarkDescriptor.hpp"
#include "Times.hpp"

#include "BillboardCloudsConverter.hpp"
#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "CubeVolume.hpp"
#include "FlowerDescriptor.hpp"
#include "FoliageDescriptor.hpp"
#include "ForestDescriptor.hpp"
#include "FruitDescriptor.hpp"
#include "JoeScanScanner.hpp"
#include "Json.hpp"
#include "LogGrader.hpp"
#include "RenderLayer.hpp"
#include "Soil.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "StrandsRenderer.hpp"
#include "Tree.hpp"
#include "TreePointCloudScanner.hpp"
#include "TreeStructor.hpp"
using namespace eco_sys_lab;

void EcoSysLabLayer::OnCreate() {
  ClassRegistry::RegisterPrivateComponent<Tree>("Tree");
  ClassRegistry::RegisterPrivateComponent<TreeStructor>("TreeStructor");
  ClassRegistry::RegisterPrivateComponent<Soil>("Soil");
  ClassRegistry::RegisterPrivateComponent<Climate>("Climate");
  ClassRegistry::RegisterPrivateComponent<LogGrader>("LogGrader");
  ClassRegistry::RegisterPrivateComponent<SpatialPlantDistributionSimulator>("SpatialPlantDistributionSimulator");

  ClassRegistry::RegisterAsset<ProceduralNoise2D>("ProceduralNoise2D", {".noise2D"});
  ClassRegistry::RegisterAsset<ProceduralNoise3D>("ProceduralNoise3D", {".noise3D"});
  ClassRegistry::RegisterAsset<BarkDescriptor>("BarkDescriptor", {".bark"});
  ClassRegistry::RegisterAsset<ForestDescriptor>("ForestDescriptor", {".forest"});
  ClassRegistry::RegisterAsset<TreeDescriptor>("TreeDescriptor", {".tree"});
  ClassRegistry::RegisterAsset<ShootDescriptor>("ShootDescriptor", {".shoot"});
  ClassRegistry::RegisterAsset<FruitDescriptor>("FruitDescriptor", {".fruit"});
  ClassRegistry::RegisterAsset<FlowerDescriptor>("FlowerDescriptor", {".flower"});
  ClassRegistry::RegisterAsset<FoliageDescriptor>("FoliageDescriptor", {".foliage"});
  ClassRegistry::RegisterAsset<SoilDescriptor>("SoilDescriptor", {".soil"});
  ClassRegistry::RegisterAsset<ClimateDescriptor>("ClimateDescriptor", {".climate"});
  ClassRegistry::RegisterAsset<RadialBoundingVolume>("RadialBoundingVolume", {".rbv"});
  ClassRegistry::RegisterAsset<CubeVolume>("CubeVolume", {".cubevolume"});
  ClassRegistry::RegisterAsset<HeightField>("HeightField", {".heightfield"});
  ClassRegistry::RegisterAsset<SoilLayerDescriptor>("SoilLayerDescriptor", {".soillayer"});
  ClassRegistry::RegisterPrivateComponent<TreePointCloudScanner>("TreePointCloudScanner");

  ClassRegistry::RegisterAsset<ForestPatch>("ForestPatch", {".forestpatch"});

  ClassRegistry::RegisterAsset<Json>("Json", {".json"});
  ClassRegistry::RegisterAsset<JoeScan>("JoeScan", {".jscan"});
  ClassRegistry::RegisterPrivateComponent<JoeScanScanner>("JoeScanScanner");

  ClassRegistry::RegisterPrivateComponent<BillboardCloudsConverter>("BillboardCloudsConverter");
  if (m_randomColors.empty()) {
    for (int i = 0; i < 20000; i++) {
      m_randomColors.emplace_back(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
    }
  }

  if (m_soilLayerColors.empty()) {
    for (int i = 0; i < 10; i++) {
      glm::vec4 color = {glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)), 1.0f};
      m_soilLayerColors.emplace_back(color);
    }
  }
  m_shootStemStrands = ProjectManager::CreateTemporaryAsset<Strands>();

  m_boundingBoxMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_foliageMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_fruitMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

  m_groundFruitMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_groundLeafMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_vectorMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_scalarMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_shadowGridParticleInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  m_lightingGridParticleInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
#pragma region Internode camera
  m_visualizationCamera = Serialization::ProduceSerializable<Camera>();

  m_visualizationCamera->OnCreate();
  m_visualizationCamera->use_clear_color = true;
  m_visualizationCamera->clear_color = glm::vec3(0.5f, 0.5f, 0.5f);
#pragma endregion

  if (const auto editorLayer = Application::GetLayer<EditorLayer>()) {
    editorLayer->RegisterEditorCamera(m_visualizationCamera);
  }
}

void EcoSysLabLayer::Visualization() {
  const auto scene = GetScene();
  const auto editorLayer = Application::GetLayer<EditorLayer>();

  const auto selectedEntity = editorLayer->GetSelectedEntity();
  if (selectedEntity != m_selectedTree) {
    if (scene->IsEntityValid(selectedEntity) && scene->HasPrivateComponent<Tree>(selectedEntity)) {
      m_selectedTree = selectedEntity;
      m_lastSelectedTreeIndex = m_selectedTree.GetIndex();
      m_needFlowUpdateForSelection = true;
      m_operatorMode = static_cast<unsigned>(OperatorMode::Select);
    } else if (m_selectedTree.GetIndex() != 0) {
      m_selectedTree = Entity();
      m_needFlowUpdateForSelection = true;
    }
    if (scene->IsEntityValid(m_selectedTree)) {
      const auto& tree = scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock();
      auto& treeVisualizer = tree->tree_visualizer;
      treeVisualizer.m_selectedInternodeHandle = -1;
      treeVisualizer.m_selectedInternodeHierarchyList.clear();
      m_operatorMode = static_cast<unsigned>(OperatorMode::Select);
    }
  }
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();

  const auto branchStrands = m_shootStemStrands.Get<Strands>();
  if (treeEntities && !treeEntities->empty()) {
    bool flowUpdated = false;
    // Tree selection
    if (m_shootVersions.size() != treeEntities->size()) {
      m_internodeSize = 0;
      m_rootNodeSize = 0;
      m_totalTime = 0.0f;
      m_shootVersions.clear();
      for (int i = 0; i < treeEntities->size(); i++) {
        m_shootVersions.emplace_back(-1);
      }
      m_needFullFlowUpdate = true;
    }
    for (int i = 0; i < treeEntities->size(); i++) {
      auto treeEntity = treeEntities->at(i);
      auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      auto& treeModel = tree->tree_model;
      if (m_shootVersions[i] != treeModel.RefShootSkeleton().GetVersion()) {
        m_shootVersions[i] = treeModel.RefShootSkeleton().GetVersion();
        m_needFullFlowUpdate = true;
      }
    }

    if (m_visualization) {
      if (m_needFullFlowUpdate) {
        UpdateFlows(treeEntities, branchStrands);
        UpdateGroundFruitAndLeaves();
        m_needFullFlowUpdate = false;
        flowUpdated = true;
      }
      if (m_needFlowUpdateForSelection) {
        UpdateFlows(treeEntities, branchStrands);
        m_needFlowUpdateForSelection = false;
        flowUpdated = true;
      }
    }
    if (flowUpdated) {
      const auto climateCandidate = FindClimate();
      if (!climateCandidate.expired()) {
        const auto climate = climateCandidate.lock();
        const auto& voxelGrid = climate->climate_model.environment_grid.voxel_grid;
        const auto numVoxels = voxelGrid.GetVoxelCount();
        {
          std::vector<ParticleInfo> particleInfos;
          particleInfos.resize(numVoxels);

          Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
            const auto coordinate = voxelGrid.GetCoordinate(i);
            particleInfos[i].instance_matrix.value =
                glm::translate(voxelGrid.GetPosition(coordinate) +
                               glm::linearRand(-glm::vec3(0.5f * voxelGrid.GetVoxelSize()),
                                               glm::vec3(0.5f * voxelGrid.GetVoxelSize()))) *
                glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(0.25f * voxelGrid.GetVoxelSize()));
            particleInfos[i].instance_color = glm::vec4(
                1.f, 1.f, 1.f, 1.f - glm::clamp(voxelGrid.Peek(static_cast<int>(i)).light_intensity, 0.0f, 1.0f));
          });
          m_shadowGridParticleInfoList->SetParticleInfos(particleInfos);
        }
        {
          std::vector<ParticleInfo> particleInfos;
          particleInfos.resize(numVoxels);

          Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
            const auto coordinate = voxelGrid.GetCoordinate(i);
            const auto& voxel = voxelGrid.Peek(coordinate);
            const auto direction = voxel.light_direction;
            auto rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
            rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
            const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
            const auto voxelSize = voxelGrid.GetVoxelSize();
            particleInfos[i].instance_matrix.value =
                glm::translate(voxelGrid.GetPosition(coordinate) +
                               glm::linearRand(-glm::vec3(0.5f * voxelGrid.GetVoxelSize()),
                                               glm::vec3(0.5f * voxelGrid.GetVoxelSize()))) *
                rotationTransform * glm::scale(glm::vec3(0.05f * voxelSize, voxelSize * 0.5f, 0.05f * voxelSize));
            if (voxelGrid.Peek(static_cast<int>(i)).light_intensity == 0.0f)
              particleInfos[i].instance_color = glm::vec4(0.0f);
            else
              particleInfos[i].instance_color = glm::vec4(
                  1.f, 1.f, 1.f, 1.f - glm::clamp(voxelGrid.Peek(static_cast<int>(i)).light_intensity, 0.0f, 1.0f));
          });
          m_lightingGridParticleInfoList->SetParticleInfos(particleInfos);
        }
      }
    }
  }
  if (m_visualization) {
    GizmoSettings gizmoSettings;
    gizmoSettings.draw_settings.blending = true;

    gizmoSettings.draw_settings.blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
    gizmoSettings.draw_settings.blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    gizmoSettings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;

    if (editorLayer && scene->IsEntityValid(m_selectedTree)) {
      const auto& tree = scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock();
      auto& treeModel = tree->tree_model;
      auto& treeVisualizer = tree->tree_visualizer;
      const auto globalTransform = scene->GetDataComponent<GlobalTransform>(m_selectedTree);
#ifdef BUILD_WITH_RAYTRACER
      const auto rayTracerLayer = Application::GetLayer<RayTracerLayer>();
#endif
      if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == KeyActionType::Release &&
          treeVisualizer.m_checkpointIteration == treeModel.CurrentIteration()) {
        static bool mayNeedGeometryGeneration = false;
        static std::vector<glm::vec2> mousePositions{};
        const auto& treeSkeleton = tree->tree_model.PeekShootSkeleton(tree->tree_visualizer.m_checkpointIteration);
        switch (static_cast<OperatorMode>(m_operatorMode)) {
          case OperatorMode::Select: {
            if (m_visualizationCameraWindowFocused) {
              if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Press) {
                if (treeVisualizer.RayCastSelection(m_visualizationCamera, m_visualizationCameraMousePosition,
                                                    treeSkeleton, globalTransform)) {
                  treeVisualizer.m_needUpdate = true;
                }
              } else if (editorLayer->GetKey(GLFW_KEY_R) == KeyActionType::Press) {
                if (treeVisualizer.m_selectedInternodeHandle > 0) {
                  treeModel.Step();
                  auto& skeleton = treeModel.RefShootSkeleton();
                  auto& pruningInternode = skeleton.RefNode(treeVisualizer.m_selectedInternodeHandle);
                  const auto childHandles = pruningInternode.PeekChildHandles();
                  for (const auto& childHandle : childHandles) {
                    treeModel.PruneInternode(childHandle);
                  }
                  pruningInternode.data.internode_length *= treeVisualizer.m_selectedInternodeLengthFactor;
                  treeModel.CalculateTransform(tree->shoot_growth_controller_, true);
                  treeVisualizer.m_selectedInternodeLengthFactor = 1.0f;
                  pruningInternode.data.buds.clear();
                  skeleton.SortLists();
                  treeVisualizer.m_checkpointIteration = treeModel.CurrentIteration();
                  treeVisualizer.m_needUpdate = true;
                  if (m_autoGenerateMeshAfterEditing) {
                    tree->GenerateGeometryEntities(m_meshGeneratorSettings, -1);
                  }
                  if (m_autoGenerateStrandsAfterEditing || m_autoGenerateStrandMeshAfterEditing) {
                    if (m_autoGenerateStrandsAfterEditing) {
                      tree->InitializeStrandRenderer();
                    }
                    if (m_autoGenerateStrandMeshAfterEditing) {
                      tree->InitializeStrandModelMeshRenderer(m_strandMeshGeneratorSettings);
                    }
                  }
                }
              } else if (editorLayer->GetKey(GLFW_KEY_T) == KeyActionType::Press) {
                if (treeVisualizer.m_selectedInternodeHandle > 0) {
                  treeModel.Step();
                  treeModel.PruneInternode(treeVisualizer.m_selectedInternodeHandle);
                  treeVisualizer.m_selectedInternodeHandle = -1;
                  treeModel.RefShootSkeleton().SortLists();
                  treeVisualizer.m_checkpointIteration = treeModel.CurrentIteration();
                  treeVisualizer.m_needUpdate = true;
                  if (m_autoGenerateMeshAfterEditing) {
                    tree->GenerateGeometryEntities(m_meshGeneratorSettings, -1);
                  }
                  if (m_autoGenerateStrandsAfterEditing || m_autoGenerateStrandMeshAfterEditing) {
                    if (m_autoGenerateStrandsAfterEditing) {
                      tree->InitializeStrandRenderer();
                    }
                    if (m_autoGenerateStrandMeshAfterEditing) {
                      tree->InitializeStrandModelMeshRenderer(m_strandMeshGeneratorSettings);
                    }
                  }
                }
              } else if (editorLayer->GetKey(GLFW_KEY_ESCAPE) == KeyActionType::Press) {
                treeVisualizer.SetSelectedNode(treeSkeleton, -1);
              }
            }
          } break;
          case OperatorMode::Rotate: {
            if (treeVisualizer.m_selectedInternodeHandle > 0) {
              if (m_visualizationCameraWindowFocused) {
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
                if (ImGui::Begin("Plant Visual")) {
                  if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
                    ImGuizmo::SetOrthographic(false);
                    ImGuizmo::SetDrawlist();
                    ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y,
                                      m_visualizationCameraResolutionX, m_visualizationCameraResolutionY);
                    glm::mat4 cameraView = glm::inverse(glm::translate(editorLayer->GetSceneCameraPosition()) *
                                                        glm::mat4_cast(editorLayer->GetSceneCameraRotation()));
                    glm::mat4 cameraProjection = m_visualizationCamera->GetProjection();
                    const auto op = ImGuizmo::OPERATION::ROTATE;
                    auto& currentSkeleton = tree->tree_model.RefShootSkeleton();
                    auto& internode = currentSkeleton.RefNode(treeVisualizer.m_selectedInternodeHandle);

                    auto transform = glm::translate(internode.info.global_position) *
                                     glm::mat4_cast(internode.data.desired_global_rotation) *
                                     glm::scale(glm::vec3(1.0f));
                    const auto treeGlobalTransform = scene->GetDataComponent<GlobalTransform>(m_selectedTree);
                    auto internodeGlobalTransform = treeGlobalTransform.value * transform;
                    ImGuizmo::Manipulate(glm::value_ptr(cameraView), glm::value_ptr(cameraProjection), op,
                                         ImGuizmo::LOCAL, glm::value_ptr(internodeGlobalTransform));
                    static bool lastGizmosUsed = false;
                    if (ImGuizmo::IsUsing()) {
                      if (!lastGizmosUsed) {
                        treeModel.Step();
                        treeVisualizer.m_checkpointIteration = treeModel.CurrentIteration();
                      }
                      treeVisualizer.m_needUpdate = true;
                      Transform newInternodeTransform{};
                      newInternodeTransform.value =
                          glm::inverse(treeGlobalTransform.value) * internodeGlobalTransform;
                      auto scaleHolder = glm::vec3(1.0f);
                      newInternodeTransform.Decompose(internode.info.global_position,
                                                      internode.data.desired_global_rotation, scaleHolder);
                      auto parentHandle = internode.GetParentHandle();
                      if (parentHandle != -1) {
                        internode.data.desired_local_rotation =
                            glm::inverse(currentSkeleton.PeekNode(parentHandle).data.desired_global_rotation) *
                            internode.data.desired_global_rotation;
                      }
                      treeModel.CalculateTransform(tree->shoot_growth_controller_, true);
                      lastGizmosUsed = true;
                    } else if (lastGizmosUsed) {
                      treeModel.CalculateTransform(tree->shoot_growth_controller_, true);
                      mayNeedGeometryGeneration = true;
                      lastGizmosUsed = false;
                      treeVisualizer.m_needUpdate = true;
                      treeModel.RefShootSkeleton().CalculateRegulatedGlobalRotation();
                    }
                  }
                  ImGui::EndChild();
                }
                ImGui::End();
                ImGui::PopStyleVar();
              }
#ifdef BUILD_WITH_RAYTRACER
              else if (rayTracerLayer) {
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
                if (ImGui::Begin("Scene (RT)")) {
                  if (ImGui::BeginChild("RaySceneRenderer", ImVec2(0, 0), false)) {
                    ImGuizmo::SetOrthographic(false);
                    ImGuizmo::SetDrawlist();
                    ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y,
                                      rayTracerLayer->GetSceneCameraResolution().x,
                                      rayTracerLayer->GetSceneCameraResolution().y);
                    glm::mat4 cameraView = glm::inverse(glm::translate(editorLayer->GetSceneCameraPosition()) *
                                                        glm::mat4_cast(editorLayer->GetSceneCameraRotation()));
                    glm::mat4 cameraProjection = m_visualizationCamera->GetProjection();
                    const auto op = ImGuizmo::OPERATION::ROTATE;
                    auto& currentSkeleton = tree->tree_model.RefShootSkeleton();
                    auto& internode = currentSkeleton.RefNode(treeVisualizer.m_selectedInternodeHandle);

                    auto transform = glm::translate(internode.info.global_position) *
                                     glm::mat4_cast(internode.data.desired_global_rotation) *
                                     glm::scale(glm::vec3(1.0f));
                    const auto treeGlobalTransform = scene->GetDataComponent<GlobalTransform>(m_selectedTree);
                    auto internodeGlobalTransform = treeGlobalTransform.value * transform;
                    ImGuizmo::Manipulate(glm::value_ptr(cameraView), glm::value_ptr(cameraProjection), op,
                                         ImGuizmo::LOCAL, glm::value_ptr(internodeGlobalTransform));
                    static bool lastGizmosUsed = false;
                    if (ImGuizmo::IsUsing()) {
                      if (!lastGizmosUsed) {
                        treeModel.Step();
                        treeVisualizer.m_checkpointIteration = treeModel.CurrentIteration();
                      }
                      treeVisualizer.m_needUpdate = true;
                      Transform newInternodeTransform{};
                      newInternodeTransform.value =
                          glm::inverse(treeGlobalTransform.value) * internodeGlobalTransform;
                      auto scaleHolder = glm::vec3(1.0f);
                      newInternodeTransform.Decompose(internode.info.global_position,
                                                      internode.data.desired_global_rotation, scaleHolder);
                      auto parentHandle = internode.GetParentHandle();
                      if (parentHandle != -1) {
                        internode.data.desired_local_rotation =
                            glm::inverse(currentSkeleton.PeekNode(parentHandle).data.desired_global_rotation) *
                            internode.data.desired_global_rotation;
                      }
                      treeModel.CalculateTransform(tree->shoot_growth_controller_, true);
                      lastGizmosUsed = true;
                    } else if (lastGizmosUsed) {
                      treeModel.CalculateTransform(tree->shoot_growth_controller_, true);
                      mayNeedGeometryGeneration = true;
                      lastGizmosUsed = false;
                      treeVisualizer.m_needUpdate = true;
                      treeModel.RefShootSkeleton().CalculateRegulatedGlobalRotation();
                    }
                  }
                  ImGui::EndChild();
                }
                ImGui::End();
                ImGui::PopStyleVar();
              }
#endif
            }
            if (m_visualizationCameraWindowFocused) {
              if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Press) {
                if (treeVisualizer.m_selectedInternodeHandle <= 0) {
                  if (treeVisualizer.RayCastSelection(m_visualizationCamera, m_visualizationCameraMousePosition,
                                                      treeSkeleton, globalTransform)) {
                    treeVisualizer.m_needUpdate = true;
                  }
                }
              } else if (editorLayer->GetKey(GLFW_KEY_T) == KeyActionType::Press) {
                if (treeVisualizer.m_selectedInternodeHandle > 0) {
                  treeModel.Step();
                  treeModel.PruneInternode(treeVisualizer.m_selectedInternodeHandle);
                  treeVisualizer.m_selectedInternodeHandle = -1;
                  treeModel.RefShootSkeleton().SortLists();
                  treeVisualizer.m_checkpointIteration = treeModel.CurrentIteration();
                  treeVisualizer.m_needUpdate = true;
                  if (m_autoGenerateMeshAfterEditing) {
                    tree->GenerateGeometryEntities(m_meshGeneratorSettings, -1);
                  }
                  if (m_autoGenerateStrandsAfterEditing || m_autoGenerateStrandMeshAfterEditing) {
                    if (m_autoGenerateStrandsAfterEditing) {
                      tree->InitializeStrandRenderer();
                    }
                    if (m_autoGenerateStrandMeshAfterEditing) {
                      tree->InitializeStrandModelMeshRenderer(m_strandMeshGeneratorSettings);
                    }
                  }
                }
              } else if (editorLayer->GetKey(GLFW_KEY_ESCAPE) == KeyActionType::Press) {
                treeVisualizer.SetSelectedNode(treeSkeleton, -1);
              }
            }
          } break;
          case OperatorMode::Prune: {
            if (m_visualizationCameraWindowFocused) {
              if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Press) {
                mousePositions.clear();
                glm::vec2 mousePosition = m_visualizationCameraMousePosition;
                const float halfX = m_visualizationCamera->GetSize().x / 2.0f;
                const float halfY = m_visualizationCamera->GetSize().y / 2.0f;
                mousePosition = {-1.0f * (mousePosition.x - halfX) / halfX, -1.0f * (mousePosition.y - halfY) / halfY};
                if (mousePosition.x > -1.0f && mousePosition.x < 1.0f && mousePosition.y > -1.0f &&
                    mousePosition.y < 1.0f) {
                  mousePositions.emplace_back(mousePosition);
                }
              } else if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Hold) {
                glm::vec2 mousePosition = m_visualizationCameraMousePosition;
                const float halfX = m_visualizationCamera->GetSize().x / 2.0f;
                const float halfY = m_visualizationCamera->GetSize().y / 2.0f;
                mousePosition = {-1.0f * (mousePosition.x - halfX) / halfX, -1.0f * (mousePosition.y - halfY) / halfY};
                if (mousePosition.x > -1.0f && mousePosition.x < 1.0f && mousePosition.y > -1.0f &&
                    mousePosition.y < 1.0f && (!mousePositions.empty() && mousePosition != mousePositions.back())) {
                  mousePositions.emplace_back(mousePosition);
                }
              } else if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Release) {
                // Once released, check if empty.
                if (!mousePositions.empty()) {
                  treeModel.Step();
                  auto& skeleton = treeModel.RefShootSkeleton();
                  if (treeVisualizer.ScreenCurveSelection(
                          [&](const SkeletonNodeHandle nodeHandle) {
                            treeModel.PruneInternode(nodeHandle);
                          },
                          mousePositions, skeleton, globalTransform)) {
                    skeleton.SortLists();
                    treeVisualizer.m_checkpointIteration = treeModel.CurrentIteration();
                    treeVisualizer.m_needUpdate = true;
                    mayNeedGeometryGeneration = true;
                  } else {
                    treeModel.Pop();
                  }
                  mousePositions.clear();
                }
              }
            }
          } break;
          case OperatorMode::Invigorate: {
            if (m_visualizationCameraWindowFocused) {
              static bool lastFrameInvigorate = false;
              if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Press) {
                if (treeVisualizer.RayCastSelection(m_visualizationCamera, m_visualizationCameraMousePosition,
                                                    treeSkeleton, globalTransform)) {
                  if (!tree->enable_history)
                    treeModel.Step();
                  treeVisualizer.m_checkpointIteration = treeModel.CurrentIteration();
                  treeVisualizer.m_needUpdate = true;
                }
              } else if (treeVisualizer.m_selectedInternodeHandle >= 0 &&
                         editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Hold) {
                const auto climateCandidate = FindClimate();
                if (!climateCandidate.expired()) {
                  climateCandidate.lock()->PrepareForGrowth();
                  if (tree->TryGrowSubTree(m_simulationSettings.delta_time, treeVisualizer.m_selectedInternodeHandle,
                                           false)) {
                    treeVisualizer.m_needUpdate = true;
                    mayNeedGeometryGeneration = true;
                  }
                }
                lastFrameInvigorate = true;
              } else if (lastFrameInvigorate && editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Release) {
                treeVisualizer.SetSelectedNode(treeSkeleton, -1);
                lastFrameInvigorate = false;
                treeVisualizer.m_needUpdate = true;
              }
            }
          } break;
          case OperatorMode::Reduce: {
            if (m_visualizationCameraWindowFocused) {
              static bool lastFrameReduce = false;
              static float targetAge = 0.0f;
              if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Press) {
                if (treeVisualizer.RayCastSelection(m_visualizationCamera, m_visualizationCameraMousePosition,
                                                    treeSkeleton, globalTransform)) {
                  if (!tree->enable_history)
                    treeModel.Step();
                  treeVisualizer.m_checkpointIteration = treeModel.CurrentIteration();
                  treeVisualizer.m_needUpdate = true;
                  if (treeVisualizer.m_selectedInternodeHandle >= 0) {
                    targetAge = tree->tree_model.GetSubTreeMaxAge(treeVisualizer.m_selectedInternodeHandle);
                  }
                }
              } else if (treeVisualizer.m_selectedInternodeHandle >= 0 &&
                         editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Hold) {
                if (tree->tree_model.Reduce(tree->shoot_growth_controller_, treeVisualizer.m_selectedInternodeHandle,
                                             targetAge)) {
                  treeVisualizer.m_needUpdate = true;
                  mayNeedGeometryGeneration = true;
                }
                targetAge -= m_reduceRate;
                lastFrameReduce = true;
              } else if (lastFrameReduce && editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Release) {
                treeVisualizer.SetSelectedNode(treeSkeleton, -1);
                lastFrameReduce = false;
                treeVisualizer.m_needUpdate = true;
              }
            }
          } break;
        }

        if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Release && mayNeedGeometryGeneration) {
          if (m_autoGenerateMeshAfterEditing) {
            tree->GenerateGeometryEntities(m_meshGeneratorSettings, -1);
          }
          if (m_autoGenerateStrandsAfterEditing || m_autoGenerateStrandMeshAfterEditing) {
            tree->BuildStrandModel();
            if (m_autoGenerateStrandsAfterEditing) {
              auto strands = tree->GenerateStrands();
              tree->InitializeStrandRenderer(strands);
            }
            if (m_autoGenerateStrandMeshAfterEditing) {
              tree->InitializeStrandModelMeshRenderer(m_strandMeshGeneratorSettings);
            }
          }
        } else if (treeVisualizer.m_needUpdate && m_autoGenerateSkeletalGraphEveryFrame) {
          tree->GenerateSkeletalGraph(m_skeletalGraphSettings, treeVisualizer.m_selectedInternodeHandle,
                                      Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"),
                                      Resources::GetResource<Mesh>("PRIMITIVE_CUBE"));
        }
        mayNeedGeometryGeneration = false;
      }
      treeVisualizer.Visualize(treeModel, globalTransform);
    }

    if (m_showShadowGrid) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
                                                 m_shadowGridParticleInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
    }
    if (m_showLightingGrid) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
                                                 m_lightingGridParticleInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
    }
    if (m_displayShootStem && !m_shootStemPoints.empty()) {
      gizmoSettings.color_mode = GizmoSettings::ColorMode::Default;
      editorLayer->DrawGizmoStrands(branchStrands, m_visualizationCamera, glm::vec4(1.0f, 1.0f, 1.0f, 0.75f),
                                    glm::mat4(1.0f), 1, gizmoSettings);
    }
    if (m_displayFruit && !m_fruitMatrices->PeekParticleInfoList().empty()) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
                                                 m_fruitMatrices, glm::mat4(1.0f), 1.0f, gizmoSettings);
    }
    gizmoSettings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
    if (m_displayFoliage && !m_foliageMatrices->PeekParticleInfoList().empty()) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_visualizationCamera,
                                                 m_foliageMatrices, glm::mat4(1.0f), 1.0f, gizmoSettings);
    }
    if (m_displayGroundLeaves && !m_groundLeafMatrices->PeekParticleInfoList().empty()) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_visualizationCamera,
                                                 m_groundLeafMatrices, glm::mat4(1.0f), 1.0f, gizmoSettings);
    }
    gizmoSettings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;

    if (m_displayGroundFruit && !m_groundFruitMatrices->PeekParticleInfoList().empty()) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
                                                 m_groundFruitMatrices, glm::mat4(1.0f), 1.0f, gizmoSettings);
    }

    if (m_displayBoundingBox && !m_boundingBoxMatrices->PeekParticleInfoList().empty()) {
      editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
                                                 m_boundingBoxMatrices, glm::mat4(1.0f), 1.0f, gizmoSettings);
    }

    gizmoSettings.color_mode = GizmoSettings::ColorMode::Default;
    if (m_displaySoil) {
      SoilVisualization();
    }
  }
}

void EcoSysLabLayer::ResetAllTrees(const std::vector<Entity>* treeEntities) {
  const auto scene = Application::GetActiveScene();
  m_simulatedTime = 0;
  if (treeEntities) {
    for (const auto& i : *treeEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
      tree->Reset();
    }
  }
  m_needFullFlowUpdate = true;
  m_totalTime = 0;
  m_internodeSize = 0;
  m_leafSize = 0;
  m_fruitSize = 0;
  m_shootStemSize = 0;
  m_rootNodeSize = 0;
  m_rootStemSize = 0;

  m_shootStemSegments.clear();
  m_shootStemPoints.clear();

  m_shootStemStrands = ProjectManager::CreateTemporaryAsset<Strands>();

  m_boundingBoxMatrices->SetParticleInfos({});
  m_foliageMatrices->SetParticleInfos({});
  m_fruitMatrices->SetParticleInfos({});

  const auto climateCandidate = FindClimate();
  if (!climateCandidate.expired()) {
    const auto climate = climateCandidate.lock();
    climate->climate_model.environment_grid = {};
  }
}

std::weak_ptr<Climate> EcoSysLabLayer::FindClimate() {
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* climateEntities = scene->UnsafeGetPrivateComponentOwnersList<Climate>();
  if (climateEntities && !climateEntities->empty()) {
    return scene->GetOrSetPrivateComponent<Climate>(climateEntities->at(0));
  }
  return {};
}

std::weak_ptr<Soil> EcoSysLabLayer::FindSoil() {
  const auto scene = Application::GetActiveScene();
  const std::vector<Entity>* soilEntities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soilEntities && !soilEntities->empty()) {
    return scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0));
  }
  return {};
}

const std::vector<glm::vec3>& EcoSysLabLayer::RandomColors() {
  return m_randomColors;
}

void EcoSysLabLayer::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  auto scene = GetScene();
  bool simulate = false;
  static bool autoTimeGrow = false;
  static float targetTime = 0.0f;
  static float extraTime = 4.f;
  if (ImGui::Begin("EcoSysLab Layer")) {
    const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (treeEntities && !treeEntities->empty()) {
      ImGui::Text("Editing");
      if (scene->IsEntityValid(m_selectedTree)) {
        const auto& tree = scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock();
        auto& treeVisualizer = tree->tree_visualizer;
        if (treeVisualizer.m_checkpointIteration == tree->tree_model.CurrentIteration()) {
          if (ImGui::TreeNodeEx("Tree Operator", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Combo("Mode", {"Select", "Rotate", "Prune", "Invigorate", "Reduce"}, m_operatorMode)) {
              treeVisualizer.m_selectedInternodeHandle = -1;
              treeVisualizer.m_selectedInternodeHierarchyList.clear();
            }
            switch (static_cast<OperatorMode>(m_operatorMode)) {
              case OperatorMode::Select:
                ImGui::Text("Press T to cut off entire node, press R to cut at point of selection.");
                break;
              case OperatorMode::Rotate:
                ImGui::Text("Press T to cut off entire node.");
                break;
              case OperatorMode::Prune:
                break;
              case OperatorMode::Invigorate:
                break;
              case OperatorMode::Reduce:
                break;
            }
            if (m_operatorMode == static_cast<unsigned>(OperatorMode::Invigorate)) {
              ImGui::DragFloat("Invigorate speed", &m_simulationSettings.delta_time, 0.01f, 0.01f, 1.0f);
            }
            if (m_operatorMode == static_cast<unsigned>(OperatorMode::Reduce)) {
              ImGui::DragFloat("Reduce speed", &m_reduceRate, 0.001f, 0.001f, 1.0f);
            }

            ImGui::TreePop();
          }
        } else {
          ImGui::Text("Go to current skeleton to enable operator!");
        }
        ImGui::Separator();
        if (ImGui::TreeNodeEx("Tree Visualizer", ImGuiTreeNodeFlags_DefaultOpen)) {
          treeVisualizer.OnInspect(tree->tree_model);
          ImGui::TreePop();
        }
      } else {
        ImGui::Text("Select a tree entity to enable editing & visualization!");
      }
      ImGui::Separator();
      ImGui::Text("Growth simulation");
      if (ImGui::TreeNode("Simulation Settings")) {
        m_simulationSettings.OnInspect(editorLayer);
        ImGui::TreePop();
      }
      if (ImGui::Button("Reset all trees")) {
        ResetAllTrees(treeEntities);
        ClearMeshes();
        ClearGroundFruitAndLeaf();
        targetTime = 0.0f;
      }
      ImGui::Text(("Simulated time: " + std::to_string(m_simulatedTime) + " years").c_str());
      ImGui::DragInt("target nodes", &m_simulationSettings.max_node_count, 500, 0, INT_MAX);
      ImGui::DragFloat("target years", &extraTime, 0.1f, m_simulatedTime, 999);
      if (autoTimeGrow) {
        if (ImGui::Button("Force stop")) {
          autoTimeGrow = false;
          targetTime = m_simulatedTime;
        }
      } else {
        if (ImGui::Button(("Grow " + std::to_string(extraTime) + " years").c_str())) {
          autoTimeGrow = true;
          targetTime += extraTime;
        }
      }
      if (ImGui::Button("Grow 1 iteration")) {
        simulate = true;
      }

      if (!m_simulationSettings.auto_clear_fruit_and_leaves && ImGui::Button("Clear ground leaves and fruits")) {
        ClearGroundFruitAndLeaf();
      }
      ImGui::Separator();
      if (ImGui::TreeNode("Skeletal graph")) {
        m_skeletalGraphSettings.OnInspect();
        ImGui::TreePop();
      }
      if (ImGui::Button("Generate Skeletal graphs")) {
        GenerateSkeletalGraphs(m_skeletalGraphSettings);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Skeletal graphs")) {
        ClearSkeletalGraphs();
      }
      ImGui::Separator();
      if (ImGui::TreeNodeEx("Mesh generation")) {
        m_meshGeneratorSettings.OnInspect(editorLayer);
        ImGui::TreePop();
      }
      if (ImGui::Button("Generate Meshes")) {
        GenerateMeshes(m_meshGeneratorSettings);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Meshes")) {
        ClearMeshes();
      }
      ImGui::Separator();
      if (ImGui::TreeNodeEx("Strand Model Mesh generation")) {
        m_strandMeshGeneratorSettings.OnInspect(editorLayer);
        ImGui::TreePop();
      }
      if (ImGui::Button("Build Strand Renderer")) {
        GenerateStrandRenderers();
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Strand Renderer")) {
        ClearStrandRenderers();
      }
      if (ImGui::Button("Generate Strand Model Meshes")) {
        GenerateStrandModelMeshes(m_strandMeshGeneratorSettings);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Strand Model Meshes")) {
        ClearStrandModelMeshes();
      }
      ImGui::Separator();

      if (ImGui::TreeNode("Auto geometry generation")) {
        ImGui::Checkbox("Auto generate mesh", &m_autoGenerateMeshAfterEditing);
        ImGui::Checkbox("Auto generate Skeletal Graph Per Frame", &m_autoGenerateSkeletalGraphEveryFrame);
        ImGui::Checkbox("Auto generate strands", &m_autoGenerateStrandsAfterEditing);
        ImGui::Checkbox("Auto generate strands mesh", &m_autoGenerateStrandMeshAfterEditing);

        ImGui::TreePop();
      }
      FileUtils::SaveFile(
          "Export all trees as OBJ", "OBJ", {".obj"},
          [&](const std::filesystem::path& path) {
            ExportAllTrees(path);
          },
          false);
      if (ImGui::TreeNodeEx("Stats")) {
        ImGui::Text("Growth time: %.4f", m_lastUsedTime);
        ImGui::Text("Total time: %.4f", m_totalTime);
        ImGui::Text("Tree count: %d", treeEntities->size());
        ImGui::Text("Total internode size: %d", m_internodeSize);
        ImGui::Text("Total shoot branch size: %d", m_shootStemSize);
        ImGui::Text("Total fruit size: %d", m_fruitSize);
        ImGui::Text("Total leaf size: %d", m_leafSize);
        ImGui::Text("Total root node size: %d", m_rootNodeSize);
        ImGui::Text("Total root branch size: %d", m_rootStemSize);
        ImGui::Text("Total ground leaf size: %d", m_leaves.size());
        ImGui::Text("Total ground fruit size: %d", m_fruits.size());
        ImGui::TreePop();
      }
    } else {
      ImGui::Text("No trees in the scene!");
      ResetAllTrees(nullptr);
      targetTime = 0.0f;
    }
    ImGui::Checkbox("Visualization", &m_visualization);
    if (m_visualization && ImGui::TreeNodeEx("Visualization settings")) {
      if (ImGui::Button("Update")) {
        m_needFullFlowUpdate = true;
      }

      ImGui::Checkbox("Display shoot stem", &m_displayShootStem);
      ImGui::Checkbox("Display fruits", &m_displayFruit);
      ImGui::Checkbox("Display foliage", &m_displayFoliage);

      ImGui::Checkbox("Display ground fruit", &m_displayGroundFruit);
      ImGui::Checkbox("Display ground leaves", &m_displayGroundLeaves);

      ImGui::Checkbox("Display Soil", &m_displaySoil);
      if (m_displaySoil && ImGui::TreeNodeEx("Soil visualization settings", ImGuiTreeNodeFlags_DefaultOpen)) {
        OnSoilVisualizationMenu();
        ImGui::TreePop();
      }
      ImGui::Checkbox("Display Bounding Box", &m_displayBoundingBox);
      ImGui::Checkbox("Show Shadow Grid", &m_showShadowGrid);
      ImGui::Checkbox("Show Lighting Direction Grid", &m_showLightingGrid);
      ImGui::TreePop();
    }
  }
  ImGui::End();
  if (simulate || autoTimeGrow) {
    Simulate(m_simulationSettings);
    if (m_simulationSettings.auto_clear_fruit_and_leaves) {
      ClearGroundFruitAndLeaf();
    }
    if (scene->IsEntityValid(m_selectedTree)) {
      auto tree = scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock();
      tree->tree_visualizer.m_checkpointIteration = tree->tree_model.CurrentIteration();
      tree->tree_visualizer.m_needUpdate = true;
      if (m_autoGenerateSkeletalGraphEveryFrame) {
        tree->GenerateSkeletalGraph(m_skeletalGraphSettings, -1, Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"),
                                    Resources::GetResource<Mesh>("PRIMITIVE_CUBE"));
      }
    }
  }
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    if (targetTime <= m_simulatedTime && autoTimeGrow) {
      autoTimeGrow = false;
      for (const auto& treeEntity : *treeEntities) {
        auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
        if (m_autoGenerateMeshAfterEditing) {
          tree->GenerateGeometryEntities(m_meshGeneratorSettings, -1);
        }
        if (m_autoGenerateStrandsAfterEditing || m_autoGenerateStrandMeshAfterEditing) {
          tree->BuildStrandModel();
          if (m_autoGenerateStrandsAfterEditing) {
            auto strands = tree->GenerateStrands();
            tree->InitializeStrandRenderer(strands);
          }
          if (m_autoGenerateStrandMeshAfterEditing) {
            tree->InitializeStrandModelMeshRenderer(m_strandMeshGeneratorSettings);
          }
        }
      }
    }
  }
#pragma region Internode debugging camera
  if (m_visualization) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
    if (ImGui::Begin("Plant Visual")) {
      if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
        ImVec2 viewPortSize;
        viewPortSize = ImGui::GetWindowSize();
        m_visualizationCameraResolutionX = viewPortSize.x;
        m_visualizationCameraResolutionY = viewPortSize.y;
        ImGui::Image(m_visualizationCamera->GetRenderTexture()->GetColorImTextureId(),
                     ImVec2(viewPortSize.x, viewPortSize.y), ImVec2(0, 1), ImVec2(1, 0));
        m_visualizationCameraMousePosition = glm::vec2(FLT_MAX, -FLT_MAX);
        auto sceneCameraRotation = editorLayer->GetSceneCameraRotation();
        auto sceneCameraPosition = editorLayer->GetSceneCameraPosition();
        if (ImGui::IsWindowFocused()) {
          m_visualizationCameraWindowFocused = true;
          bool valid = true;
          auto mp = ImGui::GetMousePos();
          auto wp = ImGui::GetWindowPos();
          m_visualizationCameraMousePosition = glm::vec2(mp.x - wp.x, mp.y - wp.y);
          if (valid) {
            static bool isDraggingPreviously = false;
            bool mouseDrag = true;
            if (m_visualizationCameraMousePosition.x < 0 || m_visualizationCameraMousePosition.y < 0 ||
                m_visualizationCameraMousePosition.x > viewPortSize.x ||
                m_visualizationCameraMousePosition.y > viewPortSize.y ||
                editorLayer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) != KeyActionType::Hold) {
              mouseDrag = false;
            }
            static float prevX = 0;
            static float prevY = 0;
            if (mouseDrag && !isDraggingPreviously) {
              prevX = m_visualizationCameraMousePosition.x;
              prevY = m_visualizationCameraMousePosition.y;
            }
            const float xOffset = m_visualizationCameraMousePosition.x - prevX;
            const float yOffset = m_visualizationCameraMousePosition.y - prevY;
            prevX = m_visualizationCameraMousePosition.x;
            prevY = m_visualizationCameraMousePosition.y;
            isDraggingPreviously = mouseDrag;
#pragma region Scene Camera Controller

            if (mouseDrag && !editorLayer->lock_camera) {
              glm::vec3 front = sceneCameraRotation * glm::vec3(0, 0, -1);
              glm::vec3 right = sceneCameraRotation * glm::vec3(1, 0, 0);
              if (editorLayer->GetKey(GLFW_KEY_W) == KeyActionType::Hold) {
                sceneCameraPosition += front * static_cast<float>(Times::DeltaTime()) * editorLayer->velocity;
              }
              if (editorLayer->GetKey(GLFW_KEY_S) == KeyActionType::Hold) {
                sceneCameraPosition -= front * static_cast<float>(Times::DeltaTime()) * editorLayer->velocity;
              }
              if (editorLayer->GetKey(GLFW_KEY_A) == KeyActionType::Hold) {
                sceneCameraPosition -= right * static_cast<float>(Times::DeltaTime()) * editorLayer->velocity;
              }
              if (editorLayer->GetKey(GLFW_KEY_D) == KeyActionType::Hold) {
                sceneCameraPosition += right * static_cast<float>(Times::DeltaTime()) * editorLayer->velocity;
              }
              if (editorLayer->GetKey(GLFW_KEY_LEFT_SHIFT) == KeyActionType::Hold) {
                sceneCameraPosition.y += editorLayer->velocity * static_cast<float>(Times::DeltaTime());
              }
              if (editorLayer->GetKey(GLFW_KEY_LEFT_CONTROL) == KeyActionType::Hold) {
                sceneCameraPosition.y -= editorLayer->velocity * static_cast<float>(Times::DeltaTime());
              }
              if (xOffset != 0.0f || yOffset != 0.0f) {
                front = glm::rotate(front, glm::radians(-xOffset * editorLayer->sensitivity), glm::vec3(0, 1, 0));
                const glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
                if ((front.y < 0.99f && yOffset < 0.0f) || (front.y > -0.99f && yOffset > 0.0f)) {
                  front = glm::rotate(front, glm::radians(-yOffset * editorLayer->sensitivity), right);
                }
                const glm::vec3 up = glm::normalize(glm::cross(right, front));
                sceneCameraRotation = glm::quatLookAt(front, up);
              }
              editorLayer->SetCameraRotation(editorLayer->GetSceneCamera(), sceneCameraRotation);
              editorLayer->SetCameraPosition(editorLayer->GetSceneCamera(), sceneCameraPosition);
            }
#pragma endregion
          }
        } else {
          m_visualizationCameraWindowFocused = false;
        }
        editorLayer->SetCameraRotation(m_visualizationCamera, sceneCameraRotation);
        editorLayer->SetCameraPosition(m_visualizationCamera, sceneCameraPosition);
      }
      ImGui::EndChild();
      auto* window = ImGui::FindWindowByName("Plant Visual");
      m_visualizationCamera->SetEnabled(!(window->Hidden && !window->Collapsed));
    }
    ImGui::End();
    ImGui::PopStyleVar();
    Visualization();
  }
#pragma endregion
}

void EcoSysLabLayer::OnSoilVisualizationMenu() {
  static bool forceUpdate;
  ImGui::Checkbox("Force Update", &forceUpdate);

  if (ImGui::Checkbox("Vector Visualization", &m_vectorEnable)) {
    if (m_vectorEnable)
      m_updateVectorMatrices = true;
  }

  if (ImGui::Checkbox("Scalar Visualization", &m_scalarEnable)) {
    if (m_scalarEnable)
      m_updateScalarMatrices = true;
  }

  if (m_vectorEnable) {
    m_updateVectorMatrices = m_updateVectorMatrices || forceUpdate;

    if (ImGui::TreeNodeEx("Vector", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Reset")) {
        m_vectorMultiplier = 50.0f;
        m_vectorBaseColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
        m_vectorSoilProperty = 4;
        m_vectorLineWidthFactor = 0.1f;
        m_vectorLineMaxWidth = 0.1f;
        m_updateVectorMatrices = true;
      }
      if (ImGui::ColorEdit4("Vector Base Color", &m_vectorBaseColor.x)) {
        m_updateVectorMatrices = true;
      }
      if (ImGui::DragFloat("Multiplier", &m_vectorMultiplier, 0.1f, 0.0f, 100.0f, "%.3f")) {
        m_updateVectorMatrices = true;
      }
      if (ImGui::DragFloat("Line Width Factor", &m_vectorLineWidthFactor, 0.01f, 0.0f, 5.0f)) {
        m_updateVectorMatrices = true;
      }
      if (ImGui::DragFloat("Max Line Width", &m_vectorLineMaxWidth, 0.01f, 0.0f, 5.0f)) {
        m_updateVectorMatrices = true;
      }
      if (ImGui::Combo("Vector Mode",
                       {"N/A", "N/A", "Water Density Gradient", "Flux", "Divergence", "N/A", "N/A", "N/A"},
                       m_vectorSoilProperty)) {
        m_updateVectorMatrices = true;
      }
      ImGui::TreePop();
    }
  }
  if (m_scalarEnable) {
    m_updateScalarMatrices = m_updateScalarMatrices || forceUpdate;

    if (m_scalarEnable && ImGui::TreeNodeEx("Scalar", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Reset")) {
        m_scalarMultiplier = 1.0f;
        m_scalarBoxSize = 0.5f;
        m_scalarMinAlpha = 0.00f;
        m_scalarBaseColor = glm::vec3(0.0f, 0.0f, 1.0f);
        m_scalarSoilProperty = 1;
        m_updateScalarMatrices = true;
      }
      if (ImGui::SliderFloat("X Depth", &m_soilCutoutXDepth, 0.0f, 1.0f)) {
        m_updateScalarMatrices = true;
      }
      if (ImGui::SliderFloat("Z Depth", &m_soilCutoutZDepth, 0.0f, 1.0f)) {
        m_updateScalarMatrices = true;
      }

      if (ImGui::TreeNodeEx("Layer colors", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int i = 0; i < 10; i++) {
          ImGui::ColorEdit4(("Layer " + std::to_string(i)).c_str(), &m_soilLayerColors[i].x);
        }
        ImGui::TreePop();
      }

      if (ImGui::ColorEdit3("Scalar Base Color", &m_scalarBaseColor.x)) {
        m_updateScalarMatrices = true;
      }
      if (ImGui::SliderFloat("Multiplier", &m_scalarMultiplier, 0.001, 10000, "%.4f", ImGuiSliderFlags_Logarithmic)) {
        m_updateScalarMatrices = true;
      }
      if (ImGui::DragFloat("Min alpha", &m_scalarMinAlpha, 0.001f, 0.0f, 1.0f)) {
        m_updateScalarMatrices = true;
      }
      if (ImGui::DragFloat("Box size", &m_scalarBoxSize, 0.001f, 0.0f, 1.0f)) {
        m_updateScalarMatrices = true;
      }
      // disable less useful visualizations to avoid clutter in the gui
      if (ImGui::Combo(
              "Scalar Mode",
              {"Blank", "Water Density", "N/A", "N/A", "N/A", "Nutrient Density", "Soil Density", "Soil Layer"},
              m_scalarSoilProperty)) {
        m_updateScalarMatrices = true;
      }
      ImGui::TreePop();
    }
  }
}

void EcoSysLabLayer::UpdateFlows(const std::vector<Entity>* treeEntities,
                                 const std::shared_ptr<Strands>& branchStrands) {
  {
    const auto scene = Application::GetActiveScene();

    m_boundingBoxMatrices->SetParticleInfos({});

    std::vector<int> branchStartIndices;
    int branchLastStartIndex = 0;
    branchStartIndices.emplace_back(branchLastStartIndex);

    std::vector<int> fruitStartIndices;
    int fruitLastStartIndex = 0;
    fruitStartIndices.emplace_back(fruitLastStartIndex);

    std::vector<int> leafStartIndices;
    int leafLastStartIndex = 0;
    leafStartIndices.emplace_back(leafLastStartIndex);

    if (treeEntities->empty()) {
      m_shootStemSegments.clear();
      m_shootStemPoints.clear();

      m_foliageMatrices->SetParticleInfos({});
      m_fruitMatrices->SetParticleInfos({});
    }
    std::vector<ParticleInfo> boundingBoxMatrices;
    for (int listIndex = 0; listIndex < treeEntities->size(); listIndex++) {
      auto treeEntity = treeEntities->at(listIndex);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      auto& treeModel = tree->tree_model;
      const auto& branchSkeleton = treeModel.RefShootSkeleton();
      const auto& branchList = branchSkeleton.PeekSortedFlowList();

      auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
      auto& [instanceMatrix, instanceColor] = boundingBoxMatrices.emplace_back();
      instanceMatrix.value =
          entityGlobalTransform.value * (glm::translate((branchSkeleton.max + branchSkeleton.min) / 2.0f) *
                                           glm::scale(branchSkeleton.max - branchSkeleton.min));
      instanceColor = glm::vec4(m_randomColors[listIndex], 0.05f);
      if (treeEntity != m_selectedTree) {
        branchLastStartIndex += branchList.size();
        branchStartIndices.emplace_back(branchLastStartIndex);

        fruitLastStartIndex += treeModel.GetFruitCount();
        fruitStartIndices.emplace_back(fruitLastStartIndex);

        leafLastStartIndex += treeModel.GetLeafCount();
        leafStartIndices.emplace_back(leafLastStartIndex);
      } else {
        branchStartIndices.emplace_back(branchLastStartIndex);
        fruitStartIndices.emplace_back(fruitLastStartIndex);
        leafStartIndices.emplace_back(leafLastStartIndex);
      }
    }

    m_boundingBoxMatrices->SetParticleInfos(boundingBoxMatrices);

    m_shootStemSegments.resize(branchLastStartIndex * 3);
    m_shootStemPoints.resize(branchLastStartIndex * 6);

    {
      std::vector<ParticleInfo> foliageMatrices;
      std::vector<ParticleInfo> fruitMatrices;
      foliageMatrices.resize(leafLastStartIndex);
      fruitMatrices.resize(fruitLastStartIndex);
      Jobs::RunParallelFor(treeEntities->size(), [&](unsigned treeIndex) {
        auto treeEntity = treeEntities->at(treeIndex);
        if (treeEntity == m_selectedTree)
          return;
        auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
        auto& treeModel = tree->tree_model;
        const auto& branchSkeleton = treeModel.RefShootSkeleton();
        const auto& branchFlowList = branchSkeleton.PeekSortedFlowList();
        const auto& internodeList = branchSkeleton.PeekSortedNodeList();
        auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
        auto branchStartIndex = branchStartIndices[treeIndex];
        for (int i = 0; i < branchFlowList.size(); i++) {
          auto& flow = branchSkeleton.PeekFlow(branchFlowList[i]);
          auto cp1 = flow.info.global_start_position;
          auto cp4 = flow.info.global_end_position;
          float distance = glm::distance(cp1, cp4);
          glm::vec3 cp0, cp2;
          if (flow.GetParentHandle() > 0) {
            cp0 = cp1 + branchSkeleton.PeekFlow(flow.GetParentHandle()).info.global_end_rotation *
                            glm::vec3(0, 0, 1) * distance / 3.0f;
            cp2 = cp1 + branchSkeleton.PeekFlow(flow.GetParentHandle()).info.global_end_rotation *
                            glm::vec3(0, 0, -1) * distance / 3.0f;
          } else {
            cp0 = cp1 + flow.info.global_start_rotation * glm::vec3(0, 0, 1) * distance / 3.0f;
            cp2 = cp1 + flow.info.global_start_rotation * glm::vec3(0, 0, -1) * distance / 3.0f;
          }
          auto cp3 = cp4 + flow.info.global_end_rotation * glm::vec3(0, 0, 1) * distance / 3.0f;
          auto cp5 = cp4 + flow.info.global_end_rotation * glm::vec3(0, 0, -1) * distance / 3.0f;

          auto& p0 = m_shootStemPoints[branchStartIndex * 6 + i * 6];
          auto& p1 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 1];
          auto& p2 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 2];
          auto& p3 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 3];
          auto& p4 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 4];
          auto& p5 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 5];
          p0.position = (entityGlobalTransform.value * glm::translate(cp0))[3];
          p1.position = (entityGlobalTransform.value * glm::translate(cp1))[3];
          p2.position = (entityGlobalTransform.value * glm::translate(cp2))[3];
          p3.position = (entityGlobalTransform.value * glm::translate(cp3))[3];
          p4.position = (entityGlobalTransform.value * glm::translate(cp4))[3];
          p5.position = (entityGlobalTransform.value * glm::translate(cp5))[3];
          if (flow.GetParentHandle() > 0) {
            p1.thickness = branchSkeleton.PeekFlow(flow.GetParentHandle()).info.end_thickness * 0.5f;
          } else {
            p1.thickness = flow.info.start_thickness * 0.5f;
          }
          p4.thickness = flow.info.end_thickness * 0.5f;

          p2.thickness = p3.thickness = (p1.thickness + p4.thickness) * 0.5f;
          p0.thickness = 2.0f * p1.thickness - p2.thickness;
          p5.thickness = 2.0f * p4.thickness - p3.thickness;

          p0.color = glm::vec4(m_randomColors[flow.data.order], 1.0f);
          p1.color = glm::vec4(m_randomColors[flow.data.order], 1.0f);
          p2.color = glm::vec4(m_randomColors[flow.data.order], 1.0f);
          p3.color = glm::vec4(m_randomColors[flow.data.order], 1.0f);
          p4.color = glm::vec4(m_randomColors[flow.data.order], 1.0f);
          p5.color = glm::vec4(m_randomColors[flow.data.order], 1.0f);

          m_shootStemSegments[branchStartIndex * 3 + i * 3] = branchStartIndex * 6 + i * 6;
          m_shootStemSegments[branchStartIndex * 3 + i * 3 + 1] = branchStartIndex * 6 + i * 6 + 1;
          m_shootStemSegments[branchStartIndex * 3 + i * 3 + 2] = branchStartIndex * 6 + i * 6 + 2;
        }
        auto leafStartIndex = leafStartIndices[treeIndex];
        auto fruitStartIndex = fruitStartIndices[treeIndex];

        int leafIndex = 0;
        int fruitIndex = 0;

        for (const auto& internodeHandle : internodeList) {
          const auto& internode = branchSkeleton.PeekNode(internodeHandle);
          const auto& internodeData = internode.data;

          for (const auto& bud : internodeData.buds) {
            if (bud.status != BudStatus::Died)
              continue;
            if (bud.reproductive_module.maturity <= 0.0f)
              continue;

            if (bud.type == BudType::Leaf) {
              foliageMatrices[leafStartIndex + leafIndex].instance_matrix.value =
                  entityGlobalTransform.value * bud.reproductive_module.transform;
              foliageMatrices[leafStartIndex + leafIndex].instance_color =
                  glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                                     glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f),
                                     1.0f - bud.reproductive_module.health),
                            1.0f);

              leafIndex++;
            } else if (bud.type == BudType::Fruit) {
              fruitMatrices[fruitStartIndex + fruitIndex].instance_matrix.value =
                  entityGlobalTransform.value * bud.reproductive_module.transform;
              fruitMatrices[fruitStartIndex + fruitIndex].instance_color =
                  glm::vec4(255 / 255.0f, 165 / 255.0f, 0 / 255.0f, 1.0f);

              fruitIndex++;
            }
          }
        }
      });
      StrandPointAttributes strandPointAttributes{};
      strandPointAttributes.normal = false;
      branchStrands->SetSegments(strandPointAttributes, m_shootStemSegments, m_shootStemPoints);
      m_foliageMatrices->SetParticleInfos(foliageMatrices);
      m_fruitMatrices->SetParticleInfos(fruitMatrices);
    }
  }
}

void EcoSysLabLayer::ClearGroundFruitAndLeaf() {
  m_fruits.clear();
  m_leaves.clear();
  UpdateGroundFruitAndLeaves();
}

void EcoSysLabLayer::UpdateGroundFruitAndLeaves() const {
  std::vector<ParticleInfo> fruitMatrices;
  fruitMatrices.resize(m_fruits.size());
  for (int i = 0; i < m_fruits.size(); i++) {
    fruitMatrices[i].instance_matrix.value = m_fruits[i].m_globalTransform.value;
    fruitMatrices[i].instance_color = glm::vec4(255 / 255.0f, 165 / 255.0f, 0 / 255.0f, 1.0f);
  }

  std::vector<ParticleInfo> leafMatrices;
  leafMatrices.resize(m_leaves.size());
  for (int i = 0; i < m_leaves.size(); i++) {
    leafMatrices[i].instance_matrix.value = m_leaves[i].m_globalTransform.value;
    leafMatrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                           glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - m_leaves[i].m_health),
                  1.0f);
  }
  m_groundFruitMatrices->SetParticleInfos(fruitMatrices);
  m_groundLeafMatrices->SetParticleInfos(leafMatrices);
}

void EcoSysLabLayer::SoilVisualization() {
  std::shared_ptr<Soil> soil;
  const auto soilCandidate = FindSoil();
  if (!soilCandidate.expired())
    soil = soilCandidate.lock();

  if (!soil)
    return;

  auto& soilModel = soil->soil_model;
  if (m_soilVersion != soilModel.m_version) {
    m_updateVectorMatrices = true;
    m_updateScalarMatrices = true;
    m_soilVersion = soilModel.m_version;
  }

  if (m_vectorEnable) {
    SoilVisualizationVector(soilModel);
  }
  if (m_scalarEnable) {
    SoilVisualizationScalar(soilModel);
  }
}

void EcoSysLabLayer::SoilVisualizationScalar(VoxelSoilModel& soilModel) {
  const auto numVoxels = soilModel.m_resolution.x * soilModel.m_resolution.y * soilModel.m_resolution.z;
  if (m_updateScalarMatrices) {
    std::vector<ParticleInfo> particleInfos;
    particleInfos.resize(numVoxels);
    Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
      const auto coordinate = soilModel.GetCoordinateFromIndex(i);
      if (static_cast<float>(coordinate.x) / soilModel.m_resolution.x < m_soilCutoutXDepth ||
          static_cast<float>(coordinate.z) / soilModel.m_resolution.z > (1.0f - m_soilCutoutZDepth)) {
        particleInfos[i].instance_matrix.value = glm::mat4(0.0f);
      } else {
        particleInfos[i].instance_matrix.value = glm::translate(soilModel.GetPositionFromCoordinate(coordinate)) *
                                                    glm::mat4_cast(glm::quat(glm::vec3(0.0f))) *
                                                    glm::scale(glm::vec3(soilModel.GetVoxelSize() * m_scalarBoxSize));
      }
    });
    auto visualize_vec3 = [&](const Field& x, const Field& y, const Field& z) {
      Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
        const auto value = glm::vec3(x[i], y[i], z[i]);
        particleInfos[i].instance_color = {
            glm::normalize(value), glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f)};
      });
    };

    auto visualize_float = [&](const Field& v) {
      Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
        const auto value = glm::vec3(v[i]);
        particleInfos[i].instance_color = {
            m_scalarBaseColor, glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f)};
      });
    };

    switch (static_cast<SoilProperty>(m_scalarSoilProperty)) {
      case SoilProperty::Blank: {
        Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
          particleInfos[i].instance_color = {m_scalarBaseColor, 0.01f};
        });
      } break;
      case SoilProperty::WaterDensity: {
        visualize_float(soilModel.m_w);
      } break;
      case SoilProperty::NutrientDensity: {
        visualize_float(soilModel.m_n);
      } break;
      case SoilProperty::SoilDensity: {
        visualize_float(soilModel.m_d);
      } break;
      case SoilProperty::SoilLayer: {
        Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
          const auto layerIndex = soilModel.m_material_id[i];
          if (layerIndex == 0)
            particleInfos[i].instance_color = glm::vec4(0.0f);
          else {
            particleInfos[i].instance_color = m_soilLayerColors[layerIndex - 1];
          }
        });
      } break;
        /*case SoilProperty::DiffusionDivergence:
        {
                visualize_vec3(soilModel.m_div_diff_x, soilModel.m_div_diff_y, soilModel.m_div_diff_z);
        }break;*/
      default: {
        Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
          particleInfos[i].instance_color = {m_scalarBaseColor, 0.01f};
        });
      } break;
    }
    m_groundFruitMatrices->SetParticleInfos(particleInfos);
  }
  m_updateScalarMatrices = false;
  const auto editorLayer = Application::GetLayer<EditorLayer>();
  GizmoSettings gizmoSettings;
  gizmoSettings.draw_settings.blending = true;
  gizmoSettings.draw_settings.blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  gizmoSettings.draw_settings.blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  gizmoSettings.draw_settings.cull_mode = VK_CULL_MODE_NONE;
  editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_scalarMatrices,
                                             glm::mat4(1.0f), 1.0f, gizmoSettings);
}

void EcoSysLabLayer::SoilVisualizationVector(VoxelSoilModel& soilModel) {
  const auto numVoxels = soilModel.m_resolution.x * soilModel.m_resolution.y * soilModel.m_resolution.z;

  if (m_updateVectorMatrices) {
    std::vector<ParticleInfo> particleInfos;
    particleInfos.resize(numVoxels);

    const auto actualVectorMultiplier = m_vectorMultiplier * soilModel.m_dx;
    switch (static_cast<SoilProperty>(m_vectorSoilProperty)) {
        /*
        case SoilProperty::WaterDensityGradient:
        {
                Jobs::ParallelFor(numVoxels, [&](unsigned i)
                        {
                                const auto targetVector = glm::vec3(soilModel.m_w_grad_x[i], soilModel.m_w_grad_y[i],
        soilModel.m_w_grad_z[i]); const auto start =
        soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)); const auto end = start + targetVector
        * actualVectorMultiplier; const auto direction = glm::normalize(end - start); glm::quat rotation =
        glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x)); rotation *=
        glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f)); const auto length = glm::distance(end, start) / 2.0f;
                                const auto width = glm::min(m_vectorLineMaxWidth, length * m_vectorLineWidthFactor);
                                const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
                                        glm::scale(glm::vec3(width, length, width));
                                particleInfos[i] = model;
                        }, results);
        }break;*/
        /*
        case SoilProperty::Divergence:
        {
                Jobs::ParallelFor(numVoxels, [&](unsigned i)
                        {
                                const auto targetVector = glm::vec3(soilModel.m_div_diff_x[i],
        soilModel.m_div_diff_y[i], soilModel.m_div_diff_z[i]); const auto start =
        soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)); const auto end = start + targetVector
        * actualVectorMultiplier; const auto direction = glm::normalize(end - start); glm::quat rotation =
        glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x)); rotation *=
        glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f)); const auto length = glm::distance(end, start) / 2.0f;
                                const auto width = glm::min(m_vectorLineMaxWidth, length * m_vectorLineWidthFactor);
                                const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
                                        glm::scale(glm::vec3(width, length, width));
                                particleInfos[i] = model;
                        }, results);
        }break;
        */
      default: {
        Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
          particleInfos[i].instance_matrix.value =
              glm::translate(soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i))) *
              glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(0.0f));
        });
      } break;
    }
    Jobs::RunParallelFor(numVoxels, [&](unsigned i) {
      particleInfos[i].instance_color = m_vectorBaseColor;
    });

    m_groundFruitMatrices->SetParticleInfos(particleInfos);
    m_updateVectorMatrices = false;
  }

  const auto editorLayer = Application::GetLayer<EditorLayer>();
  GizmoSettings gizmoSettings;
  gizmoSettings.draw_settings.blending = true;
  gizmoSettings.draw_settings.blending_src_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  gizmoSettings.draw_settings.blending_dst_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  gizmoSettings.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;

  editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), m_vectorMatrices,
                                             glm::mat4(1.0f), 1.0f, gizmoSettings);
}

float EcoSysLabLayer::GetSimulatedTime() const {
  return m_simulatedTime;
}

void EcoSysLabLayer::ExportAllTrees(const std::filesystem::path& path) const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    if (path.extension() == ".obj") {
      std::ofstream of;
      of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
      if (of.is_open()) {
        std::string start = "#Forest OBJ exporter, by Bosheng Li";
        start += "\n";
        of.write(start.c_str(), start.size());
        of.flush();
        unsigned startIndex = 1;
        if (m_meshGeneratorSettings.enable_branch) {
          unsigned treeIndex = 0;
          for (const auto& entity : *treeEntities) {
            const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
            const auto mesh = tree->GenerateBranchMesh(m_meshGeneratorSettings);
            auto& vertices = mesh->UnsafeGetVertices();
            auto& triangles = mesh->UnsafeGetTriangles();
            const auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o branch " + std::to_string(treeIndex) + "\n";
#pragma region Data collection
              for (auto i = 0; i < vertices.size(); i++) {
                auto vertexPosition = glm::vec4(vertices.at(i).position, 1.0f);
                vertexPosition = gt.value * vertexPosition;
                auto& color = vertices.at(i).color;
                data << "v " + std::to_string(vertexPosition.x) + " " + std::to_string(vertexPosition.y) + " " +
                            std::to_string(vertexPosition.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto i = 0; i < triangles.size(); i++) {
                const auto triangle = triangles[i];
                const auto f1 = triangle.x + startIndex;
                const auto f2 = triangle.y + startIndex;
                const auto f3 = triangle.z + startIndex;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              startIndex += vertices.size();
            }
            treeIndex++;
          }
        }
        if (m_meshGeneratorSettings.enable_foliage) {
          unsigned treeIndex = 0;
          for (const auto& entity : *treeEntities) {
            const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
            const auto mesh = tree->GenerateFoliageMesh(m_meshGeneratorSettings);
            auto& vertices = mesh->UnsafeGetVertices();
            auto& triangles = mesh->UnsafeGetTriangles();
            const auto gt = scene->GetDataComponent<GlobalTransform>(entity);
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o foliage " + std::to_string(treeIndex) + "\n";
#pragma region Data collection
              for (auto i = 0; i < vertices.size(); i++) {
                auto vertexPosition = glm::vec4(vertices.at(i).position, 1.0f);
                vertexPosition = gt.value * vertexPosition;
                auto& color = vertices.at(i).color;
                data << "v " + std::to_string(vertexPosition.x) + " " + std::to_string(vertexPosition.y) + " " +
                            std::to_string(vertexPosition.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto i = 0; i < triangles.size(); i++) {
                const auto triangle = triangles[i];
                const auto f1 = triangle.x + startIndex;
                const auto f2 = triangle.y + startIndex;
                const auto f3 = triangle.z + startIndex;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              startIndex += vertices.size();
            }
            treeIndex++;
          }
        }
        of.close();
      }
    }
  }
}

glm::vec2 EcoSysLabLayer::GetMouseSceneCameraPosition() const {
  return m_visualizationCameraMousePosition;
}

void EcoSysLabLayer::Simulate(const SimulationSettings& simulationSettings) {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  m_simulatedTime += simulationSettings.delta_time;
  if (treeEntities && !treeEntities->empty()) {
    float time = Times::Now();

    std::shared_ptr<Climate> climate;
    std::shared_ptr<Soil> soil;
    const auto climateCandidate = FindClimate();
    if (!climateCandidate.expired())
      climate = climateCandidate.lock();
    const auto soilCandidate = FindSoil();
    if (!soilCandidate.expired())
      soil = soilCandidate.lock();
    if (!soil) {
      EVOENGINE_ERROR("Simulation Failed! No soil in scene!");
      return;
    }
    if (!climate) {
      EVOENGINE_ERROR("Simulation Failed! No climate in scene!");
      return;
    }
    climate->climate_model.time = m_simulatedTime;

    if (simulationSettings.soil_simulation) {
      soil->soil_model.Irrigation();
      soil->soil_model.Step();
    }
    for (const auto& treeEntity : *treeEntities) {
      auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      tree->climate = climate;
      tree->soil = soil;
      tree->crown_shyness_distance = simulationSettings.crown_shyness_distance;
    }
    climate->PrepareForGrowth();
    std::vector<bool> grownStat{};
    grownStat.resize(Jobs::GetWorkerSize());
    Jobs::RunParallelFor(treeEntities->size(), [&](unsigned i, unsigned threadIndex) {
      const auto treeEntity = treeEntities->at(i);
      if (!scene->IsEntityEnabled(treeEntity))
        return;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      if (!tree->IsEnabled())
        return;
      if (tree->start_time > m_simulatedTime)
        return;
      if (simulationSettings.max_node_count > 0 &&
          tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= simulationSettings.max_node_count)
        return;
      grownStat[threadIndex] = tree->TryGrow(simulationSettings.delta_time, true);
    });

    auto heightField = soil->soil_descriptor.Get<SoilDescriptor>()->height_field.Get<HeightField>();
    for (const auto& treeEntity : *treeEntities) {
      if (!scene->IsEntityEnabled(treeEntity))
        continue;
      auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      auto treeGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
      if (!tree->IsEnabled())
        continue;
      // Collect fruit and leaves here.
      if (!simulationSettings.auto_clear_fruit_and_leaves) {
        for (const auto& fruit : tree->tree_model.RefShootSkeleton().data.dropped_fruits) {
          Fruit newFruit;
          newFruit.m_globalTransform.value = treeGlobalTransform.value * fruit.transform;

          auto position = newFruit.m_globalTransform.GetPosition();
          const auto groundHeight = heightField->GetValue({position.x, position.z});
          const auto height = position.y - groundHeight;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = groundHeight + 0.1f;
          newFruit.m_globalTransform.SetPosition(position);

          newFruit.m_maturity = fruit.maturity;
          newFruit.m_health = fruit.health;
          m_fruits.emplace_back(newFruit);
        }

        for (const auto& leaf : tree->tree_model.RefShootSkeleton().data.dropped_leaves) {
          Leaf newLeaf;
          newLeaf.m_globalTransform.value = treeGlobalTransform.value * leaf.transform;

          auto position = newLeaf.m_globalTransform.GetPosition();
          const auto groundHeight = heightField ? heightField->GetValue({position.x, position.z}) : 0.0f;
          const auto height = position.y - groundHeight;
          position.x += glm::gaussRand(0.0f, height * 0.1f);
          position.z += glm::gaussRand(0.0f, height * 0.1f);
          position.y = groundHeight + 0.1f;
          newLeaf.m_globalTransform.SetPosition(position);

          newLeaf.m_maturity = leaf.maturity;
          newLeaf.m_health = leaf.health;
          m_leaves.emplace_back(newLeaf);
        }
        tree->tree_visualizer.m_needUpdate = true;
      }
      tree->tree_model.RefShootSkeleton().data.dropped_fruits.clear();
      tree->tree_model.RefShootSkeleton().data.dropped_leaves.clear();
    }
    m_lastUsedTime = Times::Now() - time;
    m_totalTime += m_lastUsedTime;
    bool treeGrown = false;
    for (int i = 0; i < grownStat.size(); i++) {
      if (grownStat[i]) {
        treeGrown = true;
      }
    }
    if (treeGrown) {
      m_needFullFlowUpdate = true;

      int totalInternodeSize = 0;
      int totalFlowSize = 0;
      int totalRootNodeSize = 0;
      int totalRootFlowSize = 0;
      int totalLeafSize = 0;
      int totalFruitSize = 0;
      for (auto treeEntity : *treeEntities) {
        auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
        auto& treeModel = tree->tree_model;
        totalInternodeSize += treeModel.RefShootSkeleton().PeekSortedNodeList().size();
        totalFlowSize += treeModel.RefShootSkeleton().PeekSortedFlowList().size();
        totalLeafSize += treeModel.GetLeafCount();
        totalFruitSize += treeModel.GetFruitCount();
      }
      m_internodeSize = totalInternodeSize;
      m_shootStemSize = totalFlowSize;
      m_rootNodeSize = totalRootNodeSize;
      m_rootStemSize = totalRootFlowSize;
      m_leafSize = totalLeafSize;
      m_fruitSize = totalFruitSize;
    }
  }
}

void EcoSysLabLayer::Simulate() {
  Simulate(m_simulationSettings);
}

void EcoSysLabLayer::GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings) const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      if (tree->generate_mesh)
        tree->GenerateGeometryEntities(meshGeneratorSettings);
    }
  }
}

void EcoSysLabLayer::GenerateSkeletalGraphs(const SkeletalGraphSettings& skeletalGraphSettings) const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      if (tree->generate_mesh)
        tree->GenerateSkeletalGraph(m_skeletalGraphSettings, -1, Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"),
                                    Resources::GetResource<Mesh>("PRIMITIVE_CUBE"));
    }
  }
}

void EcoSysLabLayer::GenerateStrandModelProfiles() const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      if (tree->generate_mesh)
        tree->BuildStrandModel();
    }
  }
}

void EcoSysLabLayer::GenerateStrandModelMeshes(
    const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings) const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      if (tree->generate_mesh)
        tree->InitializeStrandModelMeshRenderer(strandModelMeshGeneratorSettings);
    }
  }
}

void EcoSysLabLayer::GenerateStrandRenderers() const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      if (tree->generate_mesh)
        tree->InitializeStrandRenderer();
    }
  }
}

void EcoSysLabLayer::ClearStrandRenderers() const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      tree->ClearStrandRenderer();
    }
  }
}

void EcoSysLabLayer::ClearStrandModelMeshes() const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      tree->ClearStrandModelMeshRenderer();
    }
  }
}

void EcoSysLabLayer::ClearMeshes() const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      tree->ClearGeometryEntities();
    }
  }
}

void EcoSysLabLayer::ClearSkeletalGraphs() const {
  const auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities && !treeEntities->empty()) {
    const auto copiedEntities = *treeEntities;
    for (auto treeEntity : copiedEntities) {
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      tree->ClearSkeletalGraph();
    }
  }
}

void EcoSysLabLayer::PreUpdate() {
  if (const auto editorLayer = Application::GetLayer<EditorLayer>(); !editorLayer)
    return;
  m_visualizationCamera->Resize({m_visualizationCameraResolutionX, m_visualizationCameraResolutionY});
}
