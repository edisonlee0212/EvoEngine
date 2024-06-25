#pragma once

#include "StrandModelProfile.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
template <typename ParticleData>
class StrandModelProfileSerializer {
 public:
  static void Serialize(
      YAML::Emitter& out, const StrandModelProfile<ParticleData>& strand_model_profile,
      const std::function<void(YAML::Emitter& particle_out, const ParticleData& particle_data)>& particle_func);

  static void Deserialize(
      const YAML::Node& in, StrandModelProfile<ParticleData>& strand_model_profile,
      const std::function<void(const YAML::Node& particle_in, ParticleData& particle_data)>& particle_func);
};

template <typename ParticleData>
void StrandModelProfileSerializer<ParticleData>::Serialize(
    YAML::Emitter& out, const StrandModelProfile<ParticleData>& strand_model_profile,
    const std::function<void(YAML::Emitter& particle_out, const ParticleData& particle_data)>& particle_func) {
  const auto particle_size = strand_model_profile.particles_2d_.size();
  auto color_list = std::vector<glm::vec3>(particle_size);
  auto position_list = std::vector<glm::vec2>(particle_size);
  auto last_position_list = std::vector<glm::vec2>(particle_size);
  auto acceleration_list = std::vector<glm::vec2>(particle_size);
  auto delta_position_list = std::vector<glm::vec2>(particle_size);
  auto boundary_list = std::vector<int>(particle_size);
  auto distance_to_boundary_list = std::vector<float>(particle_size);
  auto initial_position_list = std::vector<glm::vec2>(particle_size);

  auto corresponding_child_node_handle_list = std::vector<SkeletonNodeHandle>(particle_size);
  auto strand_list = std::vector<StrandHandle>(particle_size);
  auto strand_segment_handle_list = std::vector<StrandSegmentHandle>(particle_size);
  auto main_child_list = std::vector<int>(particle_size);
  auto base_list = std::vector<int>(particle_size);

  for (int particle_index = 0; particle_index < particle_size; particle_index++) {
    const auto& particle = strand_model_profile.particles_2d_[particle_index];
    color_list[particle_index] = particle.color_;
    position_list[particle_index] = particle.position_;
    last_position_list[particle_index] = particle.last_position_;
    acceleration_list[particle_index] = particle.acceleration_;
    delta_position_list[particle_index] = particle.delta_position_;

    boundary_list[particle_index] = particle.boundary_ ? 1 : 0;
    distance_to_boundary_list[particle_index] = particle.distance_to_boundary_;
    initial_position_list[particle_index] = particle.initial_position_;

    corresponding_child_node_handle_list[particle_index] = particle.corresponding_child_node_handle;
    strand_list[particle_index] = particle.strand_handle;
    strand_segment_handle_list[particle_index] = particle.strand_segment_handle;
    main_child_list[particle_index] = particle.main_child ? 1 : 0;
    base_list[particle_index] = particle.base ? 1 : 0;
  }
  if (particle_size != 0) {
    out << YAML::Key << "particles_2d_.color_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(color_list.data()),
                        color_list.size() * sizeof(glm::vec3));
    out << YAML::Key << "particles_2d_.position_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(position_list.data()),
                        position_list.size() * sizeof(glm::vec2));
    out << YAML::Key << "particles_2d_.last_position_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(last_position_list.data()),
                        last_position_list.size() * sizeof(glm::vec2));
    out << YAML::Key << "particles_2d_.acceleration_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(acceleration_list.data()),
                        acceleration_list.size() * sizeof(glm::vec2));
    out << YAML::Key << "particles_2d_.delta_position_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(delta_position_list.data()),
                        delta_position_list.size() * sizeof(glm::vec2));
    out << YAML::Key << "particles_2d_.boundary_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(boundary_list.data()),
                        boundary_list.size() * sizeof(int));
    out << YAML::Key << "particles_2d_.distance_to_boundary_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(distance_to_boundary_list.data()),
                        distance_to_boundary_list.size() * sizeof(float));
    out << YAML::Key << "particles_2d_.initial_position_" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(initial_position_list.data()),
                        initial_position_list.size() * sizeof(glm::vec2));

    out << YAML::Key << "particles_2d_.corresponding_child_node_handle" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(corresponding_child_node_handle_list.data()),
                        corresponding_child_node_handle_list.size() * sizeof(SkeletonNodeHandle));
    out << YAML::Key << "particles_2d_.strand_handle" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_list.data()),
                        strand_list.size() * sizeof(StrandHandle));
    out << YAML::Key << "particles_2d_.strand_segment_handle" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(strand_segment_handle_list.data()),
                        strand_segment_handle_list.size() * sizeof(StrandSegmentHandle));
    out << YAML::Key << "particles_2d_.main_child" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(main_child_list.data()),
                        main_child_list.size() * sizeof(int));
    out << YAML::Key << "particles_2d_.base" << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(base_list.data()), base_list.size() * sizeof(int));
  }

  out << YAML::Key << "particles_2d_.data" << YAML::Value << YAML::BeginSeq;
  for (const auto& particles_2d : strand_model_profile.particles_2d_) {
    out << YAML::BeginMap;
    { particle_func(out, particles_2d.data); }
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

template <typename ParticleData>
void StrandModelProfileSerializer<ParticleData>::Deserialize(
    const YAML::Node& in, StrandModelProfile<ParticleData>& strand_model_profile,
    const std::function<void(const YAML::Node& particle_in, ParticleData& particle_data)>& particle_func) {
  if (in["particles_2d_.color_"]) {
    auto list = std::vector<glm::vec3>();
    const auto data = in["particles_2d_.color_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec3));
    std::memcpy(list.data(), data.data(), data.size());

    strand_model_profile.particles_2d_.resize(list.size());
    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].color_ = list[i];
    }
  }

  if (in["particles_2d_.position_"]) {
    auto list = std::vector<glm::vec2>();
    const auto data = in["particles_2d_.position_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec2));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].position_ = list[i];
    }
  }

  if (in["particles_2d_.last_position_"]) {
    auto list = std::vector<glm::vec2>();
    const auto data = in["particles_2d_.last_position_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec2));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].last_position_ = list[i];
    }
  }

  if (in["particles_2d_.acceleration_"]) {
    auto list = std::vector<glm::vec2>();
    const auto data = in["particles_2d_.acceleration_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec2));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].acceleration_ = list[i];
    }
  }

  if (in["particles_2d_.delta_position_"]) {
    auto list = std::vector<glm::vec2>();
    const auto data = in["particles_2d_.delta_position_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec2));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].delta_position_ = list[i];
    }
  }

  if (in["particles_2d_.boundary_"]) {
    auto list = std::vector<int>();
    const auto data = in["particles_2d_.boundary_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].boundary_ = list[i] == 1;
    }
  }

  if (in["particles_2d_.distance_to_boundary_"]) {
    auto list = std::vector<float>();
    const auto data = in["particles_2d_.distance_to_boundary_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(float));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].distance_to_boundary_ = list[i];
    }
  }

  if (in["particles_2d_.initial_position_"]) {
    auto list = std::vector<glm::vec2>();
    const auto data = in["particles_2d_.initial_position_"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(glm::vec2));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].initial_position_ = list[i];
    }
  }

  if (in["particles_2d_.corresponding_child_node_handle"]) {
    auto list = std::vector<SkeletonNodeHandle>();
    const auto data = in["particles_2d_.corresponding_child_node_handle"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(SkeletonNodeHandle));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].corresponding_child_node_handle = list[i];
    }
  }

  if (in["particles_2d_.strand_handle"]) {
    auto list = std::vector<StrandHandle>();
    const auto data = in["particles_2d_.strand_handle"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(StrandHandle));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].strand_handle = list[i];
    }
  }

  if (in["particles_2d_.strand_segment_handle"]) {
    auto list = std::vector<StrandSegmentHandle>();
    const auto data = in["particles_2d_.strand_segment_handle"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(StrandSegmentHandle));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].strand_segment_handle = list[i];
    }
  }

  if (in["particles_2d_.main_child"]) {
    auto list = std::vector<int>();
    const auto data = in["particles_2d_.main_child"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].main_child = list[i] == 1;
    }
  }

  if (in["particles_2d_.base"]) {
    auto list = std::vector<int>();
    const auto data = in["particles_2d_.base"].as<YAML::Binary>();
    list.resize(data.size() / sizeof(int));
    std::memcpy(list.data(), data.data(), data.size());

    for (size_t i = 0; i < list.size(); i++) {
      strand_model_profile.particles_2d_[i].base = list[i] == 1;
    }
  }

  if (in["particles_2d_.data"]) {
    const auto& in_particle_data_list = in["particles_2d_.data"];
    ParticleHandle particle_handle = 0;
    for (const auto& in_particle_2d : in_particle_data_list) {
      auto& particle_2d = strand_model_profile.particles_2d_[particle_handle];
      particle_2d.handle_ = particle_handle;
      particle_func(in_particle_2d, particle_2d.data);
      particle_handle++;
    }
  }
}
}  // namespace eco_sys_lab
