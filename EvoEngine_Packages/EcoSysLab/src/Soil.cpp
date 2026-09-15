#include "Soil.hpp"

#include "Material.hpp"
#include "Mesh.hpp"

#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "HeightField.hpp"

using namespace eco_sys_lab_package;

namespace {
std::shared_ptr<Texture2D> PackMetallicRoughnessTexture(const std::shared_ptr<Texture2D>& roughness_texture,
                                                        const std::shared_ptr<Texture2D>& metallic_texture,
                                                        const float roughness_factor = 1.0f,
                                                        const float metallic_factor = 1.0f) {
  const auto source_texture = roughness_texture ? roughness_texture : metallic_texture;
  if (!source_texture) {
    return nullptr;
  }
  const auto resolution = source_texture->GetResolution();
  const auto pixel_count = static_cast<size_t>(resolution.x) * resolution.y;
  std::vector<float> roughness_data(pixel_count, roughness_factor);
  std::vector<float> metallic_data(pixel_count, metallic_factor);
  if (roughness_texture && roughness_texture->GetResolution() == resolution) {
    roughness_texture->GetRedChannelData(roughness_data);
  }
  if (metallic_texture && metallic_texture->GetResolution() == resolution) {
    metallic_texture->GetRedChannelData(metallic_data);
  }
  std::vector<glm::vec3> metallic_roughness_data(pixel_count, glm::vec3(1.0f));
  for (size_t i = 0; i < pixel_count; ++i) {
    metallic_roughness_data[i].g = roughness_data[i];
    metallic_roughness_data[i].b = metallic_data[i];
  }
  const auto metallic_roughness_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  metallic_roughness_texture->SetRgbChannelData(metallic_roughness_data, resolution);
  return metallic_roughness_texture;
}

void SetMaterialTextures(const std::shared_ptr<Material>& material, const std::shared_ptr<Texture2D>& albedo_texture,
                         const std::shared_ptr<Texture2D>& normal_texture,
                         const std::shared_ptr<Texture2D>& roughness_texture,
                         const std::shared_ptr<Texture2D>& metallic_texture) {
  material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, albedo_texture);
  material->SetTexture(&GltfShadeMaterial::normal_texture, normal_texture);
  material->SetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture,
                       PackMetallicRoughnessTexture(roughness_texture, metallic_texture));
}
}  // namespace

void Soil::RandomOffset(float min, float max) {
  if (const auto sd = soil_descriptor_ref.Get<SoilDescriptor>()) {
    sd->RandomOffset(min, max);
  }
}

Entity Soil::GenerateSurfaceQuadX(bool back_facing, float depth, const glm::vec2& min_xy, const glm::vec2 max_xy,
                                  float water_factor, float nutrient_factor) {
  auto scene = ApplicationContext::Get().GetActiveScene();
  auto quad_entity = scene->CreateEntity("Slice");
  auto material = AssetManager::CreateTemporaryAsset<Material>();
  auto albedo_tex = AssetManager::CreateTemporaryAsset<Texture2D>();
  auto normal_tex = AssetManager::CreateTemporaryAsset<Texture2D>();
  auto metallic_tex = AssetManager::CreateTemporaryAsset<Texture2D>();
  auto roughness_tex = AssetManager::CreateTemporaryAsset<Texture2D>();
  std::vector<glm::vec4> albedo_data;
  std::vector<glm::vec3> normal_data;
  std::vector<float> metallic_data;
  std::vector<float> roughness_data;
  glm::ivec2 texture_resolution;
  soil_model.GetSoilTextureSlideX(back_facing, depth, min_xy, max_xy, albedo_data, normal_data, roughness_data,
                                  metallic_data, texture_resolution, water_factor, nutrient_factor);
  albedo_tex->SetRgbaChannelData(albedo_data, texture_resolution);
  normal_tex->SetRgbChannelData(normal_data, texture_resolution);
  metallic_tex->SetRedChannelData(metallic_data, texture_resolution);
  roughness_tex->SetRedChannelData(roughness_data, texture_resolution);
  SetMaterialTextures(material, albedo_tex, normal_tex, roughness_tex, metallic_tex);
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(quad_entity).lock();
  mesh_renderer->material = material;
  material->material_data.shade_material.double_sided = 1;
  material->MarkDirty();
  mesh_renderer->mesh = Resources::GetInstance().GetPrimitives().quad;

  GlobalTransform global_transform;
  glm::vec3 scale;
  glm::vec3 position;
  glm::vec3 rotation;
  auto soil_model_size = glm::vec3(soil_model.m_resolution) * soil_model.m_dx;

  scale = glm::vec3(soil_model_size.z * (max_xy.x - min_xy.x), 1.0f, soil_model_size.y * (max_xy.y - min_xy.y));
  rotation = glm::vec3(glm::radians(90.0f), glm::radians(back_facing ? 90.0f : -90.0f), 0.0f);
  position = soil_model.m_boundingBoxMin + glm::vec3(soil_model_size.x * depth,
                                                     soil_model_size.y * (min_xy.y + max_xy.y) * 0.5f,
                                                     soil_model_size.z * (min_xy.x + max_xy.x) * 0.5f);
  global_transform.SetPosition(position);
  global_transform.SetEulerRotation(rotation);
  global_transform.SetScale(scale);
  scene->SetDataComponent(quad_entity, global_transform);
  return quad_entity;
}

Entity Soil::GenerateSurfaceQuadZ(bool back_facing, float depth, const glm::vec2& min_xy, const glm::vec2 max_xy,
                                  float water_factor, float nutrient_factor) {
  auto scene = ApplicationContext::Get().GetActiveScene();
  auto quad_entity = scene->CreateEntity("Slice");

  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(quad_entity).lock();
  auto material = AssetManager::CreateTemporaryAsset<Material>();
  auto albedo_tex = AssetManager::CreateTemporaryAsset<Texture2D>();
  auto normal_tex = AssetManager::CreateTemporaryAsset<Texture2D>();
  auto metallic_tex = AssetManager::CreateTemporaryAsset<Texture2D>();
  auto roughness_tex = AssetManager::CreateTemporaryAsset<Texture2D>();
  std::vector<glm::vec4> albedo_data;
  std::vector<glm::vec3> normal_data;
  std::vector<float> metallic_data;
  std::vector<float> roughness_data;
  glm::ivec2 texture_resolution;
  soil_model.GetSoilTextureSlideZ(back_facing, depth, min_xy, max_xy, albedo_data, normal_data, roughness_data,
                                  metallic_data, texture_resolution, water_factor, nutrient_factor);
  albedo_tex->SetRgbaChannelData(albedo_data, texture_resolution);
  normal_tex->SetRgbChannelData(normal_data, texture_resolution);
  metallic_tex->SetRedChannelData(metallic_data, texture_resolution);
  roughness_tex->SetRedChannelData(roughness_data, texture_resolution);
  SetMaterialTextures(material, albedo_tex, normal_tex, roughness_tex, metallic_tex);

  mesh_renderer->material = material;
  material->material_data.shade_material.double_sided = 1;
  material->MarkDirty();
  mesh_renderer->mesh = Resources::GetInstance().GetPrimitives().quad;

  GlobalTransform global_transform;
  glm::vec3 scale;
  glm::vec3 position;
  glm::vec3 rotation;
  auto soil_model_size = glm::vec3(soil_model.m_resolution) * soil_model.m_dx;

  scale = glm::vec3(soil_model_size.x * (max_xy.x - min_xy.x), 1.0f, soil_model_size.y * (max_xy.y - min_xy.y));
  rotation = glm::vec3(glm::radians(90.0f), glm::radians(back_facing ? 180.0f : 0.0f), 0.0f);
  position = soil_model.m_boundingBoxMin + glm::vec3(soil_model_size.x * (min_xy.x + max_xy.x) * 0.5f,
                                                     soil_model_size.y * (min_xy.y + max_xy.y) * 0.5f,
                                                     soil_model_size.z * depth);

  global_transform.SetPosition(position);
  global_transform.SetEulerRotation(rotation);
  global_transform.SetScale(scale);
  scene->SetDataComponent(quad_entity, global_transform);
  return quad_entity;
}

Entity Soil::GenerateCutOut(float x_depth, float z_depth, float water_factor, float nutrient_factor,
                            bool enable_ground_surface) {
  auto scene = ApplicationContext::Get().GetActiveScene();
  const auto combined_entity = scene->CreateEntity("CutOut");

  if (z_depth <= 0.99f) {
    auto quad1 = GenerateSurfaceQuadX(false, 0, {0, 0}, {1.0 - z_depth, 1}, water_factor, nutrient_factor);
    scene->SetParent(quad1, combined_entity);
  }
  if (z_depth >= 0.01f && x_depth <= 0.99f) {
    auto quad2 = GenerateSurfaceQuadX(true, x_depth, {1.0 - z_depth, 0}, {1, 1}, water_factor, nutrient_factor);
    scene->SetParent(quad2, combined_entity);
  }
  if (x_depth >= 0.01f) {
    auto quad3 = GenerateSurfaceQuadZ(false, 1.0 - z_depth, {0, 0}, {x_depth, 1}, water_factor, nutrient_factor);
    scene->SetParent(quad3, combined_entity);
  }
  if (x_depth <= 0.99f) {
    auto quad4 = GenerateSurfaceQuadZ(true, 1.0, {x_depth, 0}, {1, 1}, water_factor, nutrient_factor);
    scene->SetParent(quad4, combined_entity);
  }

  if (enable_ground_surface) {
    auto ground_surface = GenerateMesh(x_depth, z_depth);
    if (const auto sd = soil_descriptor_ref.Get<SoilDescriptor>()) {
      if (auto& soil_layer_descriptors = sd->soil_layer_descriptors; !soil_layer_descriptors.empty()) {
        if (auto first_descriptor = soil_layer_descriptors[0].Get<SoilLayerDescriptor>()) {
          auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(ground_surface).lock();
          auto mat = mmr->material.Get<Material>();
          SetMaterialTextures(mat, first_descriptor->albedo_texture.Get<Texture2D>(),
                              first_descriptor->normal_texture.Get<Texture2D>(),
                              first_descriptor->roughness_texture.Get<Texture2D>(),
                              first_descriptor->metallic_texture.Get<Texture2D>());
        }
      }
    }
  }
  return combined_entity;
}

Entity Soil::GenerateFullBox(float water_factor, float nutrient_factor, bool ground_surface) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto combined_entity = scene->CreateEntity("Cube");

  auto quad1 = GenerateSurfaceQuadX(false, 0, {0, 0}, {1, 1}, water_factor, nutrient_factor);
  scene->SetParent(quad1, combined_entity);

  auto quad2 = GenerateSurfaceQuadX(true, 1, {0, 0}, {1, 1}, water_factor, nutrient_factor);
  scene->SetParent(quad2, combined_entity);

  auto quad3 = GenerateSurfaceQuadZ(true, 0, {0, 0}, {1, 1}, water_factor, nutrient_factor);
  scene->SetParent(quad3, combined_entity);

  auto quad4 = GenerateSurfaceQuadZ(false, 1, {0, 0}, {1, 1}, water_factor, nutrient_factor);
  scene->SetParent(quad4, combined_entity);

  if (ground_surface) {
    auto surface = GenerateMesh(0, 0);
    if (const auto sd = soil_descriptor_ref.Get<SoilDescriptor>()) {
      if (auto& soil_layer_descriptors = sd->soil_layer_descriptors; !soil_layer_descriptors.empty()) {
        if (const auto first_descriptor = soil_layer_descriptors[0].Get<SoilLayerDescriptor>()) {
          auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(surface).lock();
          auto mat = mmr->material.Get<Material>();
          SetMaterialTextures(mat, first_descriptor->albedo_texture.Get<Texture2D>(),
                              first_descriptor->normal_texture.Get<Texture2D>(),
                              first_descriptor->roughness_texture.Get<Texture2D>(),
                              first_descriptor->metallic_texture.Get<Texture2D>());
        }
      }
    }
  }
  return combined_entity;
}

void eco_sys_lab_package::SerializeSoil(YAML::Emitter& out, const Soil& target) {
  target.soil_descriptor_ref.Save("soil_descriptor_ref", out);
}

void eco_sys_lab_package::DeserializeSoil(const YAML::Node& in, Soil& target) {
  target.soil_descriptor_ref.Load("soil_descriptor_ref", in);
  target.InitializeSoilModel();
}

void Soil::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(soil_descriptor_ref);
}

Entity Soil::GenerateMesh(float x_depth, float z_depth) {
  const auto sd = soil_descriptor_ref.Get<SoilDescriptor>();
  if (!sd) {
    EVOENGINE_ERROR("No soil descriptor!");
    return {};
  }
  const auto height_field = sd->height_field.Get<HeightField>();
  if (!height_field) {
    EVOENGINE_ERROR("No height field!");
    return {};
  }
  std::vector<Vertex> vertices;
  std::vector<glm::uvec3> triangles;
  height_field->GenerateMesh(
      glm::vec2(sd->soil_parameters.m_boundingBoxMin.x, sd->soil_parameters.m_boundingBoxMin.z),
      glm::uvec2(sd->soil_parameters.m_voxelResolution.x, sd->soil_parameters.m_voxelResolution.z),
      sd->soil_parameters.m_deltaX, vertices, triangles, x_depth, z_depth);

  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto self = GetOwner();
  Entity ground_surface_entity;
  const auto children = scene->GetChildren(self);

  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Ground Mesh") {
      ground_surface_entity = child;
      break;
    }
  }
  if (ground_surface_entity.GetIndex() != 0)
    scene->DeleteEntity(ground_surface_entity);
  ground_surface_entity = scene->CreateEntity("Ground Mesh");
  scene->SetParent(ground_surface_entity, self);

  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground_surface_entity).lock();
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  const auto material = AssetManager::CreateTemporaryAsset<Material>();
  VertexAttributes vertex_attributes{};
  vertex_attributes.tex_coord = true;
  mesh->SetVertices(vertex_attributes, vertices, triangles);
  mesh_renderer->mesh = mesh;
  mesh_renderer->material = material;

  return ground_surface_entity;
}

void Soil::InitializeSoilModel() {
  if (const auto sd = soil_descriptor_ref.Get<SoilDescriptor>()) {
    auto height_field = sd->height_field.Get<HeightField>();

    auto params = sd->soil_parameters;
    params.m_boundary_x = VoxelSoilModel::Boundary::wrap;
    params.m_boundary_y = VoxelSoilModel::Boundary::absorb;
    params.m_boundary_z = VoxelSoilModel::Boundary::wrap;

    SoilSurface soil_surface;
    std::vector<SoilLayer> soil_layers;

    if (height_field) {
      soil_surface.m_height = [height_field](const glm::vec2& position) {
        return height_field->GetValue(glm::vec2(position.x, position.y));
      };
    } else {
      soil_surface.m_height = [&](const glm::vec2& position) {
        return 0.0f;
      };
    }

    soil_model.m_materialTextureResolution = sd->texture_resolution;
    // Add top air layer
    int material_index = 0;

    soil_layers.emplace_back();
    auto& first_layer = soil_layers.back();
    first_layer.m_mat = SoilPhysicalMaterial({material_index,
                                              [](const glm::vec3& pos) {
                                                return 1.0f;
                                              },
                                              [](const glm::vec3& pos) {
                                                return 0.0f;
                                              },
                                              [](const glm::vec3& pos) {
                                                return 0.0f;
                                              },
                                              [](const glm::vec3& pos) {
                                                return 0.0f;
                                              },
                                              [](const glm::vec3& pos) {
                                                return 0.0f;
                                              }});
    first_layer.m_thickness = [](const glm::vec2& position) {
      return 0.f;
    };
    first_layer.m_mat.m_soilMaterialTexture = std::make_shared<SoilMaterialTexture>();
    first_layer.m_mat.m_soilMaterialTexture->m_color_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_color_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_color_map.end(),
              glm::vec4(62.0f / 255, 49.0f / 255, 23.0f / 255, 0.0f));
    first_layer.m_mat.m_soilMaterialTexture->m_height_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_height_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_height_map.end(), 0.1f);

    first_layer.m_mat.m_soilMaterialTexture->m_metallic_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_metallic_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_metallic_map.end(), 0.2f);

    first_layer.m_mat.m_soilMaterialTexture->m_roughness_map.resize(sd->texture_resolution.x *
                                                                    sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_roughness_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_roughness_map.end(), 0.8f);

    first_layer.m_mat.m_soilMaterialTexture->m_normal_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_normal_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_normal_map.end(), glm::vec3(0.0f, 0.0f, 1.0f));

    material_index++;
    // Add user defined layers
    auto& soil_layer_descriptors = sd->soil_layer_descriptors;
    for (int i = 0; i < sd->soil_layer_descriptors.size(); i++) {
      if (auto soil_layer_descriptor = soil_layer_descriptors[i].Get<SoilLayerDescriptor>()) {
        soil_layers.emplace_back();
        auto& soil_layer = soil_layers.back();
        soil_layer.m_mat.m_c = [=](const glm::vec3& position) {
          return soil_layer_descriptor->capacity_graph.GetValue(position);
        };
        soil_layer.m_mat.m_p = [=](const glm::vec3& position) {
          return soil_layer_descriptor->permeability_graph.GetValue(position);
        };
        soil_layer.m_mat.m_d = [=](const glm::vec3& position) {
          return soil_layer_descriptor->density_graph.GetValue(position);
        };
        soil_layer.m_mat.m_n = [=](const glm::vec3& position) {
          return soil_layer_descriptor->initial_nutrients_graph.GetValue(position);
        };
        soil_layer.m_mat.m_w = [=](const glm::vec3& position) {
          return soil_layer_descriptor->initial_water_graph.GetValue(position);
        };
        soil_layer.m_mat.m_id = material_index;
        soil_layer.m_thickness = [soil_layer_descriptor](const glm::vec2& position) {
          return soil_layer_descriptor->thickness_graph.GetValue(position);
        };
        const auto albedo = soil_layer_descriptor->albedo_texture.Get<Texture2D>();
        const auto height = soil_layer_descriptor->height_texture.Get<Texture2D>();
        const auto metallic = soil_layer_descriptor->metallic_texture.Get<Texture2D>();
        const auto normal = soil_layer_descriptor->normal_texture.Get<Texture2D>();
        const auto roughness = soil_layer_descriptor->roughness_texture.Get<Texture2D>();
        soil_layer.m_mat.m_soilMaterialTexture = std::make_shared<SoilMaterialTexture>();
        if (albedo) {
          albedo->GetRgbaChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_color_map, sd->texture_resolution.x,
                                     sd->texture_resolution.y);
          if (i == 0) {
            albedo->GetRgbaChannelData(soil_layers[0].m_mat.m_soilMaterialTexture->m_color_map,
                                       sd->texture_resolution.x, sd->texture_resolution.y);
            for (auto& value : soil_layers[0].m_mat.m_soilMaterialTexture->m_color_map)
              value.w = 0.0f;
          }
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_color_map.resize(sd->texture_resolution.x *
                                                                     sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_color_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_color_map.end(),
                    ApplicationContext::Get().GetLayer<EcoSysLabLayer>()->soil_layer_colors[material_index]);
        }
        if (height) {
          height->GetRedChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_height_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_height_map.resize(sd->texture_resolution.x *
                                                                      sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_height_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_height_map.end(), 1.0f);
        }
        if (metallic) {
          metallic->GetRedChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_metallic_map, sd->texture_resolution.x,
                                      sd->texture_resolution.y);
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_metallic_map.resize(sd->texture_resolution.x *
                                                                        sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_metallic_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_metallic_map.end(), 0.2f);
        }
        if (roughness) {
          roughness->GetRedChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_roughness_map,
                                       sd->texture_resolution.x, sd->texture_resolution.y);
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_roughness_map.resize(sd->texture_resolution.x *
                                                                         sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_roughness_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_roughness_map.end(), 0.8f);
        }
        if (normal) {
          normal->GetRgbChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_normal_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_normal_map.resize(sd->texture_resolution.x *
                                                                      sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_normal_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_normal_map.end(), glm::vec3(0, 0, 1));
        }
        material_index++;
      } else {
        soil_layer_descriptors.erase(soil_layer_descriptors.begin() + i);
        i--;
      }
    }

    // Add bottom layer
    soil_layers.emplace_back();
    soil_layers.back().m_thickness = [](const glm::vec2& position) {
      return 1000.f;
    };
    soil_layers.back().m_mat.m_id = material_index;
    soil_layers.back().m_mat.m_c = [](const glm::vec3& position) {
      return 1000.f;
    };
    soil_layers.back().m_mat.m_p = [](const glm::vec3& position) {
      return 0.0f;
    };
    soil_layers.back().m_mat.m_d = [](const glm::vec3& position) {
      return 1000.f;
    };
    soil_layers.back().m_mat.m_n = [](const glm::vec3& position) {
      return 0.0f;
    };
    soil_layers.back().m_mat.m_w = [](const glm::vec3& position) {
      return 0.0f;
    };
    soil_model.Initialize(params, soil_surface, soil_layers);
  }
}

void Soil::SplitRootTestSetup() {
  InitializeSoilModel();
  if (const auto sd = soil_descriptor_ref.Get<SoilDescriptor>()) {
    const auto height_field = sd->height_field.Get<HeightField>();
    for (int i = 0; i < soil_model.m_n.size(); i++) {
      auto position = soil_model.GetPositionFromCoordinate(soil_model.GetCoordinateFromIndex(i));
      bool under_ground = true;
      if (height_field) {
        auto height = height_field->GetValue(glm::vec2(position.x, position.z));
        if (position.y >= height)
          under_ground = false;
      }
      if (under_ground) {
        if (position.x > soil_model.m_boundingBoxMin.x &&
            position.x < soil_model.GetVoxelResolution().x * soil_model.m_dx * 0.25f + soil_model.m_boundingBoxMin.x) {
          soil_model.m_n[i] = 0.75f;
        } else if (position.x <
                   soil_model.GetVoxelResolution().x * soil_model.m_dx * 0.5f + soil_model.m_boundingBoxMin.x) {
          soil_model.m_n[i] = 0.75f;
        } else if (position.x <
                   soil_model.GetVoxelResolution().x * soil_model.m_dx * 0.75f + soil_model.m_boundingBoxMin.x) {
          soil_model.m_n[i] = 1.25f;
        } else {
          soil_model.m_n[i] = 1.25f;
        }
      } else {
        soil_model.m_n[i] = 0.0f;
      }
    }
  }
}

void Soil::FixedUpdate() {
  if (temporal_progression_) {
    if (temporal_progression_progress_ < 1.0f) {
      const auto scene = ApplicationContext::Get().GetActiveScene();
      const auto owner = GetOwner();
      for (const auto& child : scene->GetChildren(owner)) {
        if (scene->GetEntityName(child) == "CutOut") {
          scene->DeleteEntity(child);
          break;
        }
      }
      const auto cut_out_entity = GenerateCutOut(temporal_progression_progress_, 0.99f, 0, 0, true);
      scene->SetParent(cut_out_entity, owner);
      temporal_progression_progress_ += 0.01f;
    } else {
      temporal_progression_progress_ = 0;
      temporal_progression_ = false;
    }
  }
}
