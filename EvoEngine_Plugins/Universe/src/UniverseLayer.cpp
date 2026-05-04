#include "UniverseLayer.hpp"

#include "Application.hpp"
#include "Times.hpp"

using namespace universe_plugin;

void UniverseLayer::RegisterTypes(Application &application) {
  application.RegisterDataComponent<StarPosition>("StarPosition");
  application.RegisterDataComponent<SelectionStatus>("SelectionStatus");
  application.RegisterDataComponent<StarInfo>("StarInfo");
  application.RegisterDataComponent<SurfaceColor>("SurfaceColor");
  application.RegisterDataComponent<DisplayColor>("DisplayColor");
  application.RegisterDataComponent<OriginalColor>("OriginalColor");
  application.RegisterDataComponent<StarOrbitOffset>("StarOrbitOffset");
  application.RegisterDataComponent<StarOrbitProportion>("StarOrbitProportion");
  application.RegisterDataComponent<StarOrbit>("StarOrbit");
  application.RegisterDataComponent<StarClusterIndex>("StarClusterIndex");
}

void UniverseLayer::OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) {
  ImGui::Checkbox("Cast shadow", &cast_shadow);

  editor_layer->DragAndDropButton<Material>(star_material_ref, "Star material");

  ImGui::InputFloat("Time", &galaxy_time_);
  static int amount = 10000;
  ImGui::DragInt("Amount", &amount, 1, 1, 100000);
  if (amount < 1)
    amount = 1;
  if (ImGui::CollapsingHeader("Star clusters", ImGuiTreeNodeFlags_DefaultOpen)) {
    int i = 0;
    for (auto &pattern : star_cluster_patterns_) {
      i++;
      if (ImGui::TreeNodeEx((std::to_string(i) + ": " + pattern.name).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::TreeNodeEx("Properties", ImGuiTreeNodeFlags_DefaultOpen)) {
          pattern.OnInspect();
          ImGui::TreePop();
        }
        if (ImGui::Button(("Add " + std::to_string(amount) + " stars").c_str())) {
          PushStars(pattern, amount);
        }
        ImGui::TreePop();
      }
    }
  }
  if (ImGui::CollapsingHeader("Star removal", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button(("Remove " + std::to_string(amount) + " stars").c_str()))
      RandomlyRemoveStars(amount);
    if (ImGui::Button("Remove all stars"))
      ClearAllStars();
  }
  if (ImGui::CollapsingHeader("Start time control", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::DragFloat("Speed", &speed_, 1.0f, 0.0f, 40000.0f);
    ImGui::DragFloat("Star Size", &size_, 0.01f, 0.01f, 10.0f);
  }
  ImGui::Text("Status:");
  ImGui::InputFloat("Apply time", &apply_position_timer_, 0, 0, "%.5f", ImGuiInputTextFlags_ReadOnly);
  ImGui::InputFloat("Copy time", &copy_position_timer_, 0, 0, "%.5f", ImGuiInputTextFlags_ReadOnly);
  ImGui::InputFloat("Calculation time", &calc_position_result_, 0, 0, "%.5f", ImGuiInputTextFlags_ReadOnly);
}

void UniverseLayer::OnCreate() {
  particle_info_list_ref = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  const auto star_material = AssetManager::CreateTemporaryAsset<Material>();
  star_material->material_properties.emission = 3.f;
  star_material_ref = star_material;
  star_cluster_patterns_.resize(2);
  auto &star_cluster_pattern1 = star_cluster_patterns_[0];
  auto &star_cluster_pattern2 = star_cluster_patterns_[1];
  star_cluster_pattern1.star_cluster_index.m_value = 0;
  star_cluster_pattern2.star_cluster_index.m_value = 1;
  star_query_ = Entities::CreateEntityQuery();
  star_query_.SetAllFilters(StarInfo());

  star_archetype_ = Entities::CreateEntityArchetype(
      "Star", GlobalTransform(), StarClusterIndex(), StarInfo(), StarOrbit(), StarOrbitOffset(), StarOrbitProportion(),
      StarPosition(), SelectionStatus(), OriginalColor(), SurfaceColor(), DisplayColor());
  first_time_ = true;
}

void UniverseLayer::OnDestroy() {
}

void CheckLod(std::mutex &mutex, const std::shared_ptr<TerrainChunk> &chunk, const PlanetInfo &info,
              const GlobalTransform &planet_transform, const GlobalTransform &camera_transform) {
  if (glm::distance(glm::dvec3(chunk->ChunkCenterPosition(planet_transform.GetPosition(), info.radius,
                                                          planet_transform.GetRotation())),
                    glm::dvec3(camera_transform.GetPosition())) <
      info.lod_distance * info.radius / glm::pow(2, chunk->detail_level + 1)) {
    if (chunk->detail_level < info.max_lod_level) {
      chunk->Expand(mutex);
    }
  }
  if (chunk->c0)
    CheckLod(mutex, chunk->c0, info, planet_transform, camera_transform);
  if (chunk->c1)
    CheckLod(mutex, chunk->c1, info, planet_transform, camera_transform);
  if (chunk->c2)
    CheckLod(mutex, chunk->c2, info, planet_transform, camera_transform);
  if (chunk->c3)
    CheckLod(mutex, chunk->c3, info, planet_transform, camera_transform);
  if (glm::distance(glm::dvec3(chunk->ChunkCenterPosition(planet_transform.GetPosition(), info.radius,
                                                          planet_transform.GetRotation())),
                    glm::dvec3(camera_transform.GetPosition())) >
      info.lod_distance * info.radius / glm::pow(2, chunk->detail_level + 1)) {
    chunk->Collapse();
  }
}

void RenderChunk(const std::shared_ptr<TerrainChunk> &chunk, const std::shared_ptr<Material> &material,
                 const GlobalTransform &matrix, const bool receive_shadow) {
  if (chunk->active) {
    const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
    render_layer->DrawMesh(chunk->mesh, material, matrix, true);
  }
  if (chunk->children_active) {
    RenderChunk(chunk->c0, material, matrix, receive_shadow);
    RenderChunk(chunk->c1, material, matrix, receive_shadow);
    RenderChunk(chunk->c2, material, matrix, receive_shadow);
    RenderChunk(chunk->c3, material, matrix, receive_shadow);
  }
}

void UniverseLayer::Update() {
  const auto scene = GetScene();
  if (!scene)
    return;
  const std::vector<Entity> *const planet_terrain_list = scene->UnsafeGetPrivateComponentOwnersList<PlanetTerrain>();
  if (const auto main_camera = scene->main_camera.Get<Camera>(); planet_terrain_list && main_camera) {
    std::mutex mesh_gen_lock;
    const auto camera_ltw = scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner());
    for (auto planet_terrain_entity : *planet_terrain_list) {
      const auto planet_terrain = scene->GetOrSetPrivateComponent<PlanetTerrain>(planet_terrain_entity).lock();
      if (!scene->IsEntityEnabled(planet_terrain_entity) || !planet_terrain->IsEnabled())
        continue;
      auto &planet_info = planet_terrain->info_;
      auto planet_transform = scene->GetDataComponent<GlobalTransform>(planet_terrain_entity);
      auto &planet_chunks = planet_terrain->chunks_;
      // 1. Scan and expand.
      for (auto &chunk : planet_chunks) {
        // futures.push_back(_PrimaryWorkers->Share([&, this](int id) { CheckLod(meshGenLock, chunk, planetInfo,
        // planetTransform, cameraLtw); }).share());
        CheckLod(mesh_gen_lock, chunk, planet_info, planet_transform, camera_ltw);
      }
      GlobalTransform global_transform;
      global_transform.value = glm::scale(
          glm::translate(glm::mat4_cast(planet_transform.GetRotation()), glm::vec3(planet_transform.GetPosition())),
          glm::vec3(1.0f));
      if (auto material = planet_terrain->surface_material.Get<Material>()) {
        for (const auto &planet_chunk : planet_chunks) {
          RenderChunk(planet_chunk, material, global_transform, true);
        }
      }
    }
  }

  galaxy_time_ += Times::DeltaTime() * speed_;
  // This method calculate the position for each star. Remove this line if you use your own implementation.
  CalculateStarPositionSync();
  // Do not touch below functions.
  counter_++;
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    if (const auto material = star_material_ref.Get<Material>()) {
      render_layer->DrawMeshInstanced(Resources::Primitives::sphere, material, {},
                                      particle_info_list_ref.Get<ParticleInfoList>(), cast_shadow);
    }
  }
}

void UniverseLayer::PushStars(StarClusterPattern &pattern, const size_t &amount) {
  const auto scene = GetScene();
  counter_ = 0;
  const auto stars = scene->CreateEntities(star_archetype_, amount, "Star");
  for (auto i = 0; i < amount; i++) {
    auto star_entity = stars[i];
    StarOrbitProportion proportion;
    proportion.value = glm::linearRand(0.0, 1.0);
    StarInfo star_info;
    scene->SetDataComponent(star_entity, star_info);
    scene->SetDataComponent(star_entity, proportion);
    scene->SetDataComponent(star_entity, pattern.star_cluster_index);
  }
  pattern.Apply();
}

void UniverseLayer::RandomlyRemoveStars(const size_t &amount) {
  counter_ = 0;
  std::vector<Entity> stars;

  const auto scene = GetScene();

  scene->GetEntityArray(star_query_, stars);
  size_t residue = amount;
  for (const auto &i : stars) {
    if (residue > 0)
      residue--;
    else
      break;
    scene->DeleteEntity(i);
  }
}

void UniverseLayer::ClearAllStars() {
  counter_ = 0;
  std::vector<Entity> stars;
  const auto scene = GetScene();
  scene->GetEntityArray(star_query_, stars);
  for (const auto &i : stars)
    scene->DeleteEntity(i);
}

void UniverseLayer::CalculateStarPositionSync() {
  const auto scene = GetScene();
  calc_position_timer_ = Times::Now();
  // StarOrbitProportion: The relative position of the star, here it is used to calculate the speed of the star
  // around its orbit. StarPosition: The final output of this operation, records the position of the star in the
  // galaxy. StarOrbit: The orbit which contains the function for calculating the position based on current time
  // and proportion value. StarOrbitOffset: The position offset of the star, used to add irregularity to the
  // position.
  Jobs::Wait(scene->ForEach<StarOrbitProportion, StarPosition, StarOrbit, StarOrbitOffset>(
      {}, star_query_,
      [=](int i, Entity entity, const StarOrbitProportion &star_proportion, StarPosition &star_position,
          const StarOrbit &star_orbit, const StarOrbitOffset &star_orbit_offset) {
        // Code here will be exec in parallel
        star_position.value =
            star_orbit.GetPoint(star_orbit_offset.value, star_proportion.value * 360.0f + galaxy_time_, true);
      },
      false));
  const auto used_time = Times::Now() - calc_position_timer_;
  calc_position_result_ = calc_position_result_ * counter_ / (counter_ + 1) + used_time / (counter_ + 1);

  // Copy data for rendering.
  ApplyPosition();
  CopyPosition();
}

void UniverseLayer::CopyPosition() {
  const auto scene = GetScene();
  const auto particle_info_list = particle_info_list_ref.Get<ParticleInfoList>();
  const auto star_amount = scene->GetEntityAmount(star_query_);
  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(star_amount);
  Jobs::Wait(scene->ForEach<GlobalTransform, DisplayColor>(
      {}, star_query_,
      [&](int i, Entity entity, const GlobalTransform &global_transform, const DisplayColor &display_color) {
        particle_infos[i].instance_matrix.value = global_transform.value;
        particle_infos[i].instance_color = glm::vec4(display_color.value * display_color.intensity, 1.0f);
      },
      false));
  particle_info_list->SetParticleInfos(particle_infos);
}
void UniverseLayer::ApplyPosition() {
  const auto scene = GetScene();
  apply_position_timer_ = Times::Now();
  Jobs::Wait(scene->ForEach<StarPosition, GlobalTransform, Transform, SurfaceColor, DisplayColor>(
      {}, star_query_,
      [this](int i, Entity entity, const StarPosition &position, GlobalTransform &global_transform,
             Transform &transform, const SurfaceColor &surface_color, DisplayColor &display_color) {
        // Code here will be exec in parallel
        global_transform.value =
            glm::translate(glm::vec3(position.value) / 20.0f) * glm::scale(size_ * glm::vec3(1.0f));
        transform.value = global_transform.value;
        display_color.value = surface_color.value;
        display_color.intensity = surface_color.intensity;
      },
      false));
  apply_position_timer_ = Times::Now() - apply_position_timer_;
}

void StarClusterPattern::OnInspect() {
  static bool auto_apply = true;
  ImGui::Checkbox("Auto apply", &auto_apply);
  if (!auto_apply && ImGui::Button("Apply"))
    Apply();
  bool need_update = false;
  float y_spread = this->y_spread;
  float xz_spread = this->xz_spread;
  float disk_diameter = this->disk_diameter;
  float disk_eccentricity = this->disk_eccentricity;
  float core_proportion = this->core_proportion;
  float core_eccentricity = this->core_eccentricity;
  float center_diameter = this->center_diameter;
  float center_eccentricity = this->center_eccentricity;
  float disk_speed = this->disk_speed;
  float core_speed = this->core_speed;
  float center_speed = this->center_speed;
  float disk_tilt_x = this->disk_tilt_x;
  float disk_tilt_z = this->disk_tilt_z;
  float core_tilt_x = this->core_tilt_x;
  float core_tilt_z = this->core_tilt_z;
  float center_tilt_x = this->center_tilt_x;
  float center_tilt_z = this->center_tilt_z;
  float twist = this->twist;
  glm::vec3 center_offset = this->center_offset;
  glm::vec3 center_position = this->center_position;
  if (ImGui::TreeNode("Shape")) {
    if (ImGui::DragFloat("Y Spread", &y_spread, 0.001f, 0.0f, 1.0f, "%.3f")) {
      this->y_spread = y_spread;
      need_update = true;
    }
    if (ImGui::DragFloat("XZ Spread", &xz_spread, 0.001f, 0.0f, 1.0f, "%.3f")) {
      this->xz_spread = xz_spread;
      need_update = true;
    }

    if (ImGui::DragFloat("Disk size", &disk_diameter, 1.0f, 1.0f, 10000.0f)) {
      this->disk_diameter = disk_diameter;
      need_update = true;
    }
    if (ImGui::DragFloat("Disk eccentricity", &disk_eccentricity, 0.01f, 0.0f, 1.0f)) {
      this->disk_eccentricity = disk_eccentricity;
      need_update = true;
    }
    if (ImGui::DragFloat("Core proportion", &core_proportion, 0.01f, 0.0f, 1.0f)) {
      this->core_proportion = core_proportion;
      need_update = true;
    }
    if (ImGui::DragFloat("Core eccentricity", &core_eccentricity, 0.01f, 0, 1)) {
      this->core_eccentricity = core_eccentricity;
      need_update = true;
    }
    if (ImGui::DragFloat("Center size", &center_diameter, 1.0f, 0, 9999)) {
      this->center_diameter = center_diameter;
      need_update = true;
    }
    if (ImGui::DragFloat("Center eccentricity", &center_eccentricity, 0.01f, 0, 1)) {
      this->center_eccentricity = center_eccentricity;
      need_update = true;
    }
    ImGui::TreePop();
    if (ImGui::DragFloat3("Center offset", &center_offset.x, 1.0f, -10000.0f, 10000.0f)) {
      center_offset = center_offset;
      need_update = true;
    }
    if (ImGui::DragFloat3("Center position", &center_position.x, 1.0f, -10000.0f, 10000.0f)) {
      center_position = center_position;
      need_update = true;
    }
  }
  if (ImGui::TreeNode("Movement")) {
    if (ImGui::DragFloat("Disk speed", &disk_speed, 0.1f, -100, 100)) {
      this->disk_speed = disk_speed;
      need_update = true;
    }
    if (ImGui::DragFloat("Core speed", &core_speed, 0.1f, -100, 100)) {
      this->core_speed = core_speed;
      need_update = true;
    }
    if (ImGui::DragFloat("Center speed", &center_speed, 0.1f, -100, 100)) {
      this->center_speed = center_speed;
      need_update = true;
    }
    if (ImGui::DragFloat("Disk X tilt", &disk_tilt_x, 1.0f, -180.0f, 180.0f)) {
      this->disk_tilt_x = disk_tilt_x;
      need_update = true;
    }
    if (ImGui::DragFloat("Disk Z tilt", &disk_tilt_z, 1.0f, -180.0f, 180.0f)) {
      this->disk_tilt_z = disk_tilt_z;
      need_update = true;
    }
    if (ImGui::DragFloat("Core X tilt", &core_tilt_x, 1.0f, -180.0f, 180.0f)) {
      this->core_tilt_x = core_tilt_x;
      need_update = true;
    }
    if (ImGui::DragFloat("Core Z tilt", &core_tilt_z, 1.0f, -180.0f, 180.0f)) {
      this->core_tilt_z = core_tilt_z;
      need_update = true;
    }
    if (ImGui::DragFloat("Center X tilt", &center_tilt_x, 1.0f, -180.0f, 180.0f)) {
      this->center_tilt_x = center_tilt_x;
      need_update = true;
    }
    if (ImGui::DragFloat("Center Z tilt", &center_tilt_z, 1.0f, -180.0f, 180.0f)) {
      this->center_tilt_z = center_tilt_z;
      need_update = true;
    }
    if (ImGui::DragFloat("Twist", &twist, 1.0f, -720.0f, 720.0f)) {
      this->twist = twist;
      need_update = true;
    }
    ImGui::TreePop();
  }
  bool color_update = false;
  if (ImGui::TreeNode("Rendering")) {
    if (ImGui::ColorEdit3("Disk Color", &this->disk_color.x, 0.1))
      color_update = true;
    if (ImGui::DragFloat("Disk Color Intensity", &this->disk_emission_intensity, 0.01f, 1.0f, 10.0f))
      color_update = true;
    if (ImGui::ColorEdit3("Core Color", &this->core_color.x, 0.1))
      color_update = true;
    if (ImGui::DragFloat("Core Color Intensity", &this->core_emission_intensity, 0.01f, 1.0f, 10.0f))
      color_update = true;
    if (ImGui::ColorEdit3("Center Color", &this->center_color.x, 0.1))
      color_update = true;
    if (ImGui::DragFloat("Center Color Intensity", &this->center_emission_intensity, 0.01f, 1.0f, 10.0f))
      color_update = true;
    ImGui::TreePop();
  }

  if (need_update) {
    Apply(true);
  } else if (color_update) {
    Apply(true, true);
  }
}

void StarClusterPattern::Apply(const bool &force_update_all_stars, const bool &only_update_colors) {
  SetAb();
  Jobs::Wait(ApplicationContext::Get()
                 .GetActiveScene()
                 ->ForEach<StarInfo, StarClusterIndex, StarOrbit, StarOrbitOffset, StarOrbitProportion, SurfaceColor>(
                     {}, [&](int i, Entity entity, StarInfo &star_info, const StarClusterIndex &star_cluster_index,
                             StarOrbit &star_orbit, StarOrbitOffset &star_orbit_offset,
                             StarOrbitProportion &star_orbit_proportion, SurfaceColor &surface_color) {
                       if (!force_update_all_stars && star_info.initialized)
                         return;
                       if (star_cluster_index.m_value != this->star_cluster_index.m_value)
                         return;
                       star_info.initialized = true;
                       const auto proportion = star_orbit_proportion.value;
                       if (!only_update_colors) {
                         star_orbit_offset = GetOrbitOffset(proportion);
                         star_orbit = GetOrbit(proportion);
                       }
                       surface_color.value = GetColor(proportion);
                       surface_color.intensity = GetIntensity(proportion);
                     }));
}
