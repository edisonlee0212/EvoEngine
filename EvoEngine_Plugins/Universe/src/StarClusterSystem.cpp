#include "StarClusterSystem.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
#include "Serialization.hpp"
#include "Resources.hpp"
#include "ClassRegistry.hpp"
#include "Graphics.hpp"
#include "Jobs.hpp"
#include "Particles.hpp"
#include "Times.hpp"
using namespace Universe;

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
  Jobs::Wait(Application::GetActiveScene()
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

bool StarClusterSystem::OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) {
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
  return false;
}

void StarClusterSystem::CalculateStarPositionSync() {
  auto scene = GetScene();
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
  use_front_ = true;
  ApplyPosition();
  CopyPosition();
}

void StarClusterSystem::ApplyPosition() {
  auto scene = GetScene();
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

void StarClusterSystem::CopyPosition(const bool &reverse) {
  auto scene = GetScene();
  bool check = reverse ? !use_front_ : use_front_;
  auto matrices = check ? scene->GetOrSetPrivateComponent<Particles>(renderer_front_.Get())
                              .lock()
                              ->particle_info_list.Get<ParticleInfoList>()
                        : scene->GetOrSetPrivateComponent<Particles>(renderer_back_.Get())
                              .lock()
                              ->particle_info_list.Get<ParticleInfoList>();
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
  matrices->SetParticleInfos(particle_infos);
}
DataComponentRegistration<StarPosition> sp_registry("StarPosition");
DataComponentRegistration<SelectionStatus> ss_registry("SelectionStatus");
DataComponentRegistration<StarInfo> si_registry("StarInfo");
DataComponentRegistration<SurfaceColor> sc_registry("SurfaceColor");
DataComponentRegistration<DisplayColor> dp_registry("DisplayColor");
DataComponentRegistration<OriginalColor> oc_registry("OriginalColor");
DataComponentRegistration<StarOrbitOffset> soo_registry("StarOrbitOffset");
DataComponentRegistration<StarOrbitProportion> sop_registry("StarOrbitProportion");
DataComponentRegistration<StarOrbit> so_registry("StarOrbit");
DataComponentRegistration<StarClusterIndex> sci_recistry("StarClusterIndex");
void StarClusterSystem::OnCreate() {
  Enable();
}

void StarClusterSystem::Update() {
  auto scene = GetScene();
  galaxy_time_ += Times::DeltaTime() * speed_;

  // This method calculate the position for each star. Remove this line if you use your own implementation.
  CalculateStarPositionSync();

  // Do not touch below functions.
  counter_++;
  /*
  Gizmos::DrawGizmoMeshInstancedColored(
      DefaultResources::Primitives::Cube,
      use_front_ ? m_frontColors : m_backColors,
      use_front_ ? scene->GetOrSetPrivateComponent<Particles>(renderer_front_.Get()).lock()->m_matrices->m_matrices
                 : scene->GetOrSetPrivateComponent<Particles>(renderer_back_.Get()).lock()->m_matrices->m_matrices,
      glm::mat4(1.0f),
      1.0f);
      */
}

void StarClusterSystem::PushStars(StarClusterPattern &pattern, const size_t &amount) {
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

void StarClusterSystem::RandomlyRemoveStars(const size_t &amount) {
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

void StarClusterSystem::ClearAllStars() {
  counter_ = 0;
  std::vector<Entity> stars;
  const auto scene = GetScene();
  scene->GetEntityArray(star_query_, stars);
  for (const auto &i : stars)
    scene->DeleteEntity(i);
}

void StarClusterSystem::FixedUpdate() {
}

void StarClusterSystem::OnEnable() {
  first_time_ = true;
}
void StarClusterSystem::Start() {
  const auto scene = GetScene();
  if (renderer_front_.Get().GetIndex() == 0 && renderer_back_.Get().GetIndex() == 0) {
    renderer_front_ = scene->CreateEntity("Renderer 1");
    GlobalTransform ltw;
    ltw.SetScale(glm::vec3(1.0f));
    const auto imr = scene->GetOrSetPrivateComponent<Particles>(renderer_front_.Get()).lock();
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    imr->material.Set<Material>(material);
    imr->cast_shadow = false;
    material->material_properties.emission = 3.0f;
    imr->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");

    scene->SetDataComponent(renderer_front_.Get(), ltw);

    renderer_back_ = scene->CreateEntity("Renderer 2");
    ltw.SetScale(glm::vec3(1.0f));
    const auto imr2 = scene->GetOrSetPrivateComponent<Particles>(renderer_back_.Get()).lock();
    imr2->material = imr->material;

    scene->SetDataComponent(renderer_back_.Get(), ltw);
  }
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
void StarClusterSystem::Serialize(YAML::Emitter &out) const {
  out << YAML::Key << "speed_" << YAML::Value << speed_;
  out << YAML::Key << "size_" << YAML::Value << size_;
  out << YAML::Key << "galaxy_time_" << YAML::Value << galaxy_time_;

  renderer_front_.Save("renderer_front_", out);
  renderer_back_.Save("renderer_back_", out);
}
void StarClusterSystem::Deserialize(const YAML::Node &in) {
  speed_ = in["speed_"].as<float>();
  size_ = in["size_"].as<float>();
  galaxy_time_ = in["galaxy_time_"].as<float>();

  renderer_front_.Load("renderer_front_", in);
  renderer_back_.Load("renderer_back_", in);
}