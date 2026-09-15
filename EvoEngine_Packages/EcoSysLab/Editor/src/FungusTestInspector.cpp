#include <glm/glm.hpp>
#include "Application.hpp"
#include "EcoSysLabObjectInspectors.hpp"
#include "EditorLayer.hpp"
#include "FungusTest.hpp"
#include "Scene.hpp"
#include "Shader.hpp"
#include "Times.hpp"
#include "Transform.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

bool FungusTestInspector::Inspect(InspectorContext& context, FungusTest& target) {
  const auto& editor_layer = context.editor_layer;
  ImGui::Checkbox("Enable Update", &target.update);
  if (target.update) {
    target.DownloadGpuResults();
    target.update = false;
  }

  if (!particle_info_list) {
    particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }
  std::vector<ParticleInfo> particle_infos(target.num_nodes);
  // const auto time = ApplicationContext::Get().GetTimes().Now();
  Jobs::RunParallelFor(particle_infos.size(), [&](size_t i) {
    auto& particle_info = particle_infos[i];
    particle_info.instance_matrix.SetPosition(
        glm::vec3(target.Tree_Graph.nodes[i].x, target.Tree_Graph.nodes[i].y, target.Tree_Graph.nodes[i].z));
    // particle_info.instance_color = glm::vec4(0.55f * (RW[i]),
    //                                          0.7f * (1.f - (RW[i])) + 0.3f,
    //                                          0.15f * (RW[i]),
    //                                          0.5f
    //);
    particle_info.instance_color = glm::vec4(0.55f + 0.45f * (1.f - target.HL[i]), 0.3f + 0.7f * (1.f - target.HL[i]),
                                             0.15f + 0.85 * (1.f - target.HL[i]), 1.f);
  });

  particle_info_list->SetParticleInfos(particle_infos);
  GizmoSettings gizmo_settings{};
  gizmo_settings.draw_settings.blending = false;
  gizmo_settings.depth_test = true;
  gizmo_settings.depth_write = true;

  editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().sphere,
                                              editor_layer->GetSceneCamera(), particle_info_list, glm::mat4(1), 0.02f,
                                              gizmo_settings);
  return false;
}
