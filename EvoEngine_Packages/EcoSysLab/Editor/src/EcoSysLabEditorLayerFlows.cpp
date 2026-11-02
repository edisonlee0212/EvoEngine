#include "EcoSysLabEditorLayer.hpp"
#include "EcoSysLabSystem.hpp"

#include "Application.hpp"

#include <algorithm>

using namespace eco_sys_lab_package;

void EcoSysLabEditorLayer::UpdateFlows(const std::vector<Entity>* tree_entities,
                                       const std::shared_ptr<Strands>& branch_strands, const bool selection_only) {
  {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const bool build_stems = !Platform::MeshShaderEnabled() && tree_visualization_settings_.display_shoot_stem;
    const bool build_leaves = !selection_only && tree_visualization_settings_.display_foliage;
    const bool build_flowers = !selection_only && tree_visualization_settings_.display_flowers;
    const bool build_fruits = !selection_only && tree_visualization_settings_.display_fruits;

    std::vector<int> branch_start_indices;
    int branch_last_start_index = 0;
    branch_start_indices.emplace_back(branch_last_start_index);

    std::vector<int> fruit_start_indices;
    int fruit_last_start_index = 0;
    fruit_start_indices.emplace_back(fruit_last_start_index);

    std::vector<int> leaf_start_indices;
    int leaf_last_start_index = 0;
    leaf_start_indices.emplace_back(leaf_last_start_index);

    std::vector<int> flower_start_indices;
    int flower_last_start_index = 0;
    flower_start_indices.emplace_back(flower_last_start_index);
    std::vector<ParticleInfo> bounding_box_matrices;
    for (int list_index = 0; list_index < tree_entities->size(); list_index++) {
      auto tree_entity = tree_entities->at(list_index);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      auto& tree_model = tree->shoot_model;
      const auto& branch_skeleton = tree_model.RefShootSkeleton();
      const auto& branch_list = branch_skeleton.PeekSortedFlowList();
      const bool enabled = scene->IsEntityEnabled(tree_entity) && tree->IsEnabled();

      auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
      if (!selection_only && tree_visualization_settings_.display_bounding_box && enabled &&
          !branch_skeleton.PeekSortedNodeList().empty()) {
        auto& [instanceMatrix, instanceColor] = bounding_box_matrices.emplace_back();
        instanceMatrix.value =
            entity_global_transform.value * (glm::translate((branch_skeleton.max + branch_skeleton.min) / 2.0f) *
                                             glm::scale(branch_skeleton.max - branch_skeleton.min));
        instanceColor = glm::vec4(random_colors_[list_index % random_colors_.size()], 0.05f);
      }

      // Simulation counters are not restored when loading a saved skeleton.
      const auto active = [](const auto& organ) {
        return organ.status != OrganStatus::Inactive;
      };
      if (enabled && (build_leaves || build_flowers || build_fruits))
        for (const auto handle : branch_skeleton.PeekSortedNodeList()) {
          const auto& data = branch_skeleton.PeekNode(handle).data;
          if (build_leaves)
            leaf_last_start_index += static_cast<int>(std::count_if(data.leaves.begin(), data.leaves.end(), active));
          if (build_flowers)
            flower_last_start_index +=
                static_cast<int>(std::count_if(data.flowers.begin(), data.flowers.end(), active));
          if (build_fruits)
            fruit_last_start_index += static_cast<int>(std::count_if(data.fruits.begin(), data.fruits.end(), active));
        }
      fruit_start_indices.emplace_back(fruit_last_start_index);
      leaf_start_indices.emplace_back(leaf_last_start_index);
      flower_start_indices.emplace_back(flower_last_start_index);

      if (build_stems && enabled && tree_entity != selected_tree) {
        branch_last_start_index += branch_list.size();
        branch_start_indices.emplace_back(branch_last_start_index);
      } else {
        branch_start_indices.emplace_back(branch_last_start_index);
      }
    }

    if (!selection_only)
      bounding_box_matrices_->SetParticleInfos(bounding_box_matrices);

    shoot_stem_segments_.resize(3 * branch_last_start_index);
    shoot_stem_points_.resize(6 * branch_last_start_index);

    {
      std::vector<ParticleInfo> foliage_matrices;
      std::vector<ParticleInfo> flower_matrices;
      std::vector<ParticleInfo> fruit_matrices;
      foliage_matrices.resize(leaf_last_start_index);
      flower_matrices.resize(flower_last_start_index);
      fruit_matrices.resize(fruit_last_start_index);
      Jobs::RunParallelFor(tree_entities->size(), [&](size_t tree_index) {
        auto tree_entity = tree_entities->at(tree_index);
        auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        if (!scene->IsEntityEnabled(tree_entity) || !tree->IsEnabled())
          return;
        auto& tree_model = tree->shoot_model;
        const auto& branch_skeleton = tree_model.RefShootSkeleton();
        const auto& branch_flow_list = branch_skeleton.PeekSortedFlowList();
        auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
        auto branch_start_index = branch_start_indices[tree_index];

        auto leaf_start_index = leaf_start_indices[tree_index];
        auto fruit_start_index = fruit_start_indices[tree_index];
        auto flower_start_index = flower_start_indices[tree_index];
        int leaf_index = 0;
        int fruit_index = 0;
        int flower_index = 0;

        const auto& sorted_internode_list = branch_skeleton.PeekSortedNodeList();
        if (build_leaves || build_flowers || build_fruits)
          for (const auto& internode_handle : sorted_internode_list) {
            const auto& internode_data = branch_skeleton.PeekNode(internode_handle).data;
            for (const auto& leaf : internode_data.leaves) {
              if (build_leaves && leaf.status != OrganStatus::Inactive) {
                glm::mat4 leaf_transform =
                    glm::translate(leaf.position) * glm::mat4_cast(leaf.rotation) * glm::scale(leaf.scale * .5f);
                foliage_matrices[leaf_start_index + leaf_index].instance_matrix.value =
                    entity_global_transform.value * leaf_transform;
                foliage_matrices[leaf_start_index + leaf_index].instance_color =
                    glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                                       glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - leaf.health),
                              0.5f);
                leaf_index++;
              }
            }

            for (const auto& flower : internode_data.flowers) {
              if (build_flowers && flower.status != OrganStatus::Inactive) {
                glm::mat4 flower_transform =
                    glm::translate(flower.position) * glm::mat4_cast(flower.rotation) * glm::scale(flower.scale);
                flower_matrices[flower_start_index + flower_index].instance_matrix.value =
                    entity_global_transform.value * flower_transform;
                flower_matrices[flower_start_index + flower_index].instance_color =
                    glm::vec4(glm::mix(glm::vec3(255 / 255.0f, 255 / 255.0f, 255 / 255.0f),
                                       glm::vec3(255 / 255.0f, 192 / 255.0f, 203 / 255.0f), flower.maturity),
                              0.75f);
                flower_index++;
              }
            }

            for (const auto& fruit : internode_data.fruits) {
              if (build_fruits && fruit.status != OrganStatus::Inactive) {
                glm::mat4 fruit_transform =
                    glm::translate(fruit.position) * glm::mat4_cast(fruit.rotation) * glm::scale(fruit.scale * .25f);
                fruit_matrices[fruit_start_index + fruit_index].instance_matrix.value =
                    entity_global_transform.value * fruit_transform;
                fruit_matrices[fruit_start_index + fruit_index].instance_color =
                    glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 255 / 255.0f, 152 / 255.0f),
                                       glm::vec3(255 / 255.0f, 165 / 255.0f, 0 / 255.0f), fruit.maturity),
                              0.75f);
                fruit_index++;
              }
            }
          }
        if (!build_stems || tree_entity == selected_tree)
          return;
        for (int i = 0; i < branch_flow_list.size(); i++) {
          auto& flow = branch_skeleton.PeekFlow(branch_flow_list[i]);
          auto cp1 = flow.info.global_start_position;
          auto cp4 = flow.info.global_end_position;
          float distance = glm::distance(cp1, cp4);
          glm::vec3 cp0, cp2;
          if (flow.GetParentHandle() > 0) {
            cp0 = cp1 + branch_skeleton.PeekFlow(flow.GetParentHandle()).info.global_end_rotation * glm::vec3(0, 0, 1) *
                            distance / 3.0f;
            cp2 = cp1 + branch_skeleton.PeekFlow(flow.GetParentHandle()).info.global_end_rotation *
                            glm::vec3(0, 0, -1) * distance / 3.0f;
          } else {
            cp0 = cp1 + flow.info.global_start_rotation * glm::vec3(0, 0, 1) * distance / 3.0f;
            cp2 = cp1 + flow.info.global_start_rotation * glm::vec3(0, 0, -1) * distance / 3.0f;
          }
          auto cp3 = cp4 + flow.info.global_end_rotation * glm::vec3(0, 0, 1) * distance / 3.0f;
          auto cp5 = cp4 + flow.info.global_end_rotation * glm::vec3(0, 0, -1) * distance / 3.0f;

          auto& p0 = shoot_stem_points_[branch_start_index * 6 + i * 6];
          auto& p1 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 1];
          auto& p2 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 2];
          auto& p3 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 3];
          auto& p4 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 4];
          auto& p5 = shoot_stem_points_[branch_start_index * 6 + i * 6 + 5];
          p0.position = (entity_global_transform.value * glm::translate(cp0))[3];
          p1.position = (entity_global_transform.value * glm::translate(cp1))[3];
          p2.position = (entity_global_transform.value * glm::translate(cp2))[3];
          p3.position = (entity_global_transform.value * glm::translate(cp3))[3];
          p4.position = (entity_global_transform.value * glm::translate(cp4))[3];
          p5.position = (entity_global_transform.value * glm::translate(cp5))[3];
          if (flow.GetParentHandle() > 0) {
            p1.thickness = branch_skeleton.PeekFlow(flow.GetParentHandle()).info.end_thickness * 0.5f;
          } else {
            p1.thickness = flow.info.start_thickness * 0.5f;
          }
          p4.thickness = flow.info.end_thickness * 0.5f;

          p2.thickness = p3.thickness = (p1.thickness + p4.thickness) * 0.5f;
          p0.thickness = 2.0f * p1.thickness - p2.thickness;
          p5.thickness = 2.0f * p4.thickness - p3.thickness;

          p0.color = p1.color = p2.color = p3.color = p4.color = p5.color =
              glm::vec4(random_colors_[static_cast<size_t>(flow.info.order) % random_colors_.size()], 1.0f);

          shoot_stem_segments_[branch_start_index * 3 + i * 3] = branch_start_index * 6 + i * 6;
          shoot_stem_segments_[branch_start_index * 3 + i * 3 + 1] = branch_start_index * 6 + i * 6 + 1;
          shoot_stem_segments_[branch_start_index * 3 + i * 3 + 2] = branch_start_index * 6 + i * 6 + 2;
        }
      });
      StrandPointAttributes strand_point_attributes{};
      strand_point_attributes.normal = false;
      branch_strands->SetSegments(strand_point_attributes, shoot_stem_segments_, shoot_stem_points_);
      if (!selection_only) {
        foliage_matrices_->SetParticleInfos(foliage_matrices);
        fruit_matrices_->SetParticleInfos(fruit_matrices);
        flower_matrices_->SetParticleInfos(flower_matrices);
      }
    }
  }
}

void EcoSysLabEditorLayer::UpdateGroundFruitAndLeaves() const {
  const auto scene = GetScene();
  if (!scene)
    return;
  const auto system = scene->GetSystem<EcoSysLabSystem>();
  if (!system)
    return;
  const auto& fruits_ = system->interactions.fruits;
  const auto& flowers_ = system->interactions.flowers;
  const auto& leaves_ = system->interactions.leaves;

  std::vector<ParticleInfo> fruit_matrices;
  fruit_matrices.resize(tree_visualization_settings_.display_ground_fruits ? fruits_.size() : 0);
  for (int i = 0; i < fruit_matrices.size(); i++) {
    fruit_matrices[i].instance_matrix.value = fruits_[i].global_transform.value;
    fruit_matrices[i].instance_matrix.SetScale(fruit_matrices[i].instance_matrix.GetScale() * 0.25f);
    fruit_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 255 / 255.0f, 152 / 255.0f),
                           glm::vec3(255 / 255.0f, 165 / 255.0f, 0 / 255.0f), fruits_[i].fruit_maturity),
                  0.75f);
  }
  std::vector<ParticleInfo> flower_matrices;
  flower_matrices.resize(tree_visualization_settings_.display_ground_flowers ? flowers_.size() : 0);
  for (int i = 0; i < flower_matrices.size(); i++) {
    flower_matrices[i].instance_matrix.value = flowers_[i].global_transform.value;
    flower_matrices[i].instance_matrix.SetScale(flower_matrices[i].instance_matrix.GetScale() * 0.5f);
    flower_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(255 / 255.0f, 255 / 255.0f, 255 / 255.0f),
                           glm::vec3(255 / 255.0f, 192 / 255.0f, 203 / 255.0f), flowers_[i].flower_maturity),
                  0.75f);
  }
  std::vector<ParticleInfo> leaf_matrices;
  leaf_matrices.resize(tree_visualization_settings_.display_ground_leaves ? leaves_.size() : 0);
  for (int i = 0; i < leaf_matrices.size(); i++) {
    leaf_matrices[i].instance_matrix.value = leaves_[i].global_transform.value;
    leaf_matrices[i].instance_matrix.SetScale(leaf_matrices[i].instance_matrix.GetScale() * 0.5f);
    leaf_matrices[i].instance_color =
        glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                           glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - leaves_[i].leaf_health),
                  0.5f);
  }
  ground_fruit_matrices_->SetParticleInfos(fruit_matrices);
  ground_leaf_matrices_->SetParticleInfos(leaf_matrices);
  ground_flower_matrices_->SetParticleInfos(flower_matrices);
}
