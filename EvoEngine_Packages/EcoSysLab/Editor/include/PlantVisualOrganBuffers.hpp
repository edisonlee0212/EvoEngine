#pragma once

#include "GeometryStorage.hpp"
#include "ShootGrowthData.hpp"

namespace eco_sys_lab_package {
struct PlantVisualOrganBuffers {
  std::vector<evo_engine::ParticleInfo> foliage, flowers, fruits;

  void Clear() {
    foliage.clear();
    flowers.clear();
    fruits.clear();
  }

  void Build(const ShootSkeleton& skeleton, const glm::mat4& transform, bool build_leaves, bool build_flowers,
             bool build_fruits) {
    Clear();
    const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
    if (build_leaves || build_flowers || build_fruits)
      for (const auto& internode_handle : sorted_internode_list) {
        const auto& internode_data = skeleton.PeekNode(internode_handle).data;
        if (build_leaves)
          for (const auto& leaf : internode_data.leaves) {
            if (leaf.status != OrganStatus::Inactive) {
              foliage.emplace_back();
              auto leaf_transform = glm::scale(glm::mat4_cast(leaf.rotation), leaf.scale * .5f);
              leaf_transform[3] = glm::vec4(leaf.position, 1.0f);
              foliage.back().instance_matrix.value = transform * leaf_transform;
              foliage.back().instance_color =
                  glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
                                     glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f), 1.0f - leaf.health),
                            0.5f);
            }
          }

        if (build_flowers)
          for (const auto& flower : internode_data.flowers) {
            if (flower.status != OrganStatus::Inactive) {
              flowers.emplace_back();
              auto flower_transform = glm::scale(glm::mat4_cast(flower.rotation), flower.scale);
              flower_transform[3] = glm::vec4(flower.position, 1.0f);
              flowers.back().instance_matrix.value = transform * flower_transform;
              flowers.back().instance_color =
                  glm::vec4(glm::mix(glm::vec3(255 / 255.0f, 255 / 255.0f, 255 / 255.0f),
                                     glm::vec3(255 / 255.0f, 192 / 255.0f, 203 / 255.0f), flower.maturity),
                            0.75f);
            }
          }

        if (build_fruits)
          for (const auto& fruit : internode_data.fruits) {
            if (fruit.status != OrganStatus::Inactive) {
              fruits.emplace_back();
              auto fruit_transform = glm::scale(glm::mat4_cast(fruit.rotation), fruit.scale * .25f);
              fruit_transform[3] = glm::vec4(fruit.position, 1.0f);
              fruits.back().instance_matrix.value = transform * fruit_transform;
              fruits.back().instance_color =
                  glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 255 / 255.0f, 152 / 255.0f),
                                     glm::vec3(255 / 255.0f, 165 / 255.0f, 0 / 255.0f), fruit.maturity),
                            0.75f);
            }
          }
      }
  }

  void Append(const PlantVisualOrganBuffers& tree) {
    foliage.insert(foliage.end(), tree.foliage.begin(), tree.foliage.end());
    flowers.insert(flowers.end(), tree.flowers.begin(), tree.flowers.end());
    fruits.insert(fruits.end(), tree.fruits.begin(), tree.fruits.end());
  }
};
}  // namespace eco_sys_lab_package
