#include "DynamicStrandsPhysicsInspectors.hpp"
#include "Shader.hpp"
#include "imgui.h"
using namespace eco_sys_lab_package;
using namespace evo_engine;
bool eco_sys_lab_package::InspectDsLeafDrop(InspectorContext& context, DsLeafDrop& target) {
  bool changed = false;
  if (ImGui::Checkbox("Enabled", &target.enabled)) {
    changed = true;
  }
  if (ImGui::DragFloat("Ground height", &target.ground_height, 0.01f, -100.0f, 100.0f))
    changed = true;

  if (ImGui::DragFloat("Rotation correction strength", &target.rotation_correction_strength, 0.001f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Air resistance strength", &target.air_resistance_strength, 0.01f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Disturbance strength", &target.disturbance_strength, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat3("Disturbance frequency", &target.disturbance_frequency.x, 0.01f, 0.01f, 10.0f))
    changed = true;
  return changed;
}
bool eco_sys_lab_package::InspectDsAttraction(InspectorContext& context, DsAttraction& target) {
  bool changed = false;
  if (ImGui::DragFloat("Multiplier", &target.distance_multiplier, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  return changed;
}
bool eco_sys_lab_package::InspectDsSnow(InspectorContext& context, DsSnow& target) {
  bool changed = false;
  if (ImGui::Checkbox("Enabled", &target.enabled)) {
    changed = true;
  }
  if (ImGui::DragFloat("Snow intensity", &target.snow_intensity, 0.0001f, -0.002f, 0.002f, "%.4f")) {
    changed = true;
  }
  if (ImGui::SliderFloat("Snow retain ratio after break", &target.snow_retain_ratio, 0.0f, 1.0f)) {
    changed = true;
  }
  return changed;
}
bool eco_sys_lab_package::InspectDsWind(InspectorContext& context, DsWind& target) {
  bool changed = false;
  if (ImGui::Checkbox("Enabled", &target.enabled)) {
    changed = true;
  }
  if (ImGui::DragFloat3("Main force", &target.main_force.x, 0.001f, -1.f, 1.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Turbulence strength", &target.turbulence_strength, 0.01f, 0.f, 2.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Turbulence directional frequency", &target.turbulence_direction_frequency, 0.01f, -100.f,
                       100.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Turbulence speed frequency", &target.turbulence_speed_frequency, 0.01f, -100.f, 100.f)) {
    changed = true;
  }
  return changed;
}
bool eco_sys_lab_package::InspectDsStiffRod(InspectorContext& context, DsStiffRod& target) {
  bool changed = false;
  if (ImGui::TreeNode("StiffRod")) {
    if (ImGui::Checkbox("Enable", &target.enabled))
      changed = true;
    if (target.enabled) {
      if (ImGui::DragInt("Sub iteration", &target.sub_iteration, 1, 1, 100))
        changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}
bool eco_sys_lab_package::InspectDsBundle(InspectorContext& context, DsBundle& target) {
  bool changed = false;
  if (ImGui::TreeNode("Random Bundle")) {
    constexpr const char* mode_names[] = {"Legacy", "Coupled XPBD", "Hybrid"};
    ImGui::Text("Solver mode: %s", mode_names[static_cast<int>(target.solver_settings.mode)]);
    if (ImGui::Checkbox("Enable", &target.enabled))
      changed = true;
    if (target.enabled) {
      if (ImGui::Checkbox("Enable bundle position", &target.enable_bundle_position))
        changed = true;
      if (ImGui::Checkbox("Enable bundle rotation", &target.enable_bundle_rotation))
        changed = true;
      if (ImGui::Checkbox("Enable bend twist", &target.enable_bend_twist))
        changed = true;
      if (ImGui::Checkbox("Enable stretch shear", &target.enable_stretch_shear))
        changed = true;
      if (ImGui::Checkbox("Enable connections", &target.enable_connections))
        changed = true;
    }
    if (ImGui::DragInt("Sub iteration", &target.sub_iteration, 1, 1, 100)) {
      target.solver_settings.legacy_iterations = target.sub_iteration;
      changed = true;
    }
    if (ImGui::DragInt("Pair iterations", &target.solver_settings.pair_iterations, 1, 1, 100))
      changed = true;
    if (ImGui::DragInt("Coarse iterations", &target.solver_settings.coarse_iterations, 1, 1, 100))
      changed = true;
    if (ImGui::DragFloat("Position compliance scale", &target.solver_settings.position_compliance_scale, 0.01f, 0.f,
                         100.f))
      changed = true;
    if (ImGui::DragFloat("Bending compliance scale", &target.solver_settings.bending_compliance_scale, 0.01f, 0.f,
                         100.f))
      changed = true;
    if (ImGui::DragFloat("Torsion compliance scale", &target.solver_settings.torsion_compliance_scale, 0.01f, 0.f,
                         100.f))
      changed = true;
    if (ImGui::SliderFloat("Shape matching strength", &target.solver_settings.shape_matching_strength, 0.f, 1.f))
      changed = true;
    if (ImGui::DragInt("Skip size", &target.skip_size, 1, 1, 100))
      changed = true;
    if (ImGui::DragFloat("Over relaxation", &target.over_relaxation, 0.01f, 1, 10.f))
      changed = true;
    if (ImGui::DragFloat("Bend Twist over relaxation", &target.bend_twist_over_relaxation, 0.01f, 1, 10.f))
      changed = true;

    if (ImGui::Button("Recompile")) {
      {
        std::shared_ptr<Shader> shader;
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateShearStretchCorrections.slang");
        target.stretch_shear_pipeline = std::make_shared<ComputePipeline>();
        target.stretch_shear_pipeline->compute_shader = shader;
        target.stretch_shear_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = target.stretch_shear_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(DsBundle::RandomBundleShearStretchConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        target.stretch_shear_pipeline->Initialize();
      }

      {
        std::shared_ptr<Shader> shader;
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBendTwistCorrections.slang");
        target.bend_twist_pipeline = std::make_shared<ComputePipeline>();
        target.bend_twist_pipeline->compute_shader = shader;
        target.bend_twist_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = target.bend_twist_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(DsBundle::RandomBundleBendTwistConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        target.bend_twist_pipeline->Initialize();
      }

      {
        std::shared_ptr<Shader> shader;
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundlePositionCorrections.slang");
        target.bundle_position_pipeline = std::make_shared<ComputePipeline>();
        target.bundle_position_pipeline->compute_shader = shader;
        target.bundle_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = target.bundle_position_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(DsBundle::RandomBundleConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        target.bundle_position_pipeline->Initialize();
      }

      {
        std::shared_ptr<Shader> shader;
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundleRotationCorrections.slang");
        target.bundle_rotation_pipeline = std::make_shared<ComputePipeline>();
        target.bundle_rotation_pipeline->compute_shader = shader;
        target.bundle_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = target.bundle_rotation_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(DsBundle::RandomBundleConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        target.bundle_rotation_pipeline->Initialize();
      }

      {
        std::shared_ptr<Shader> shader;
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyRotationCorrections.slang");

        target.apply_rotation_pipeline = std::make_shared<ComputePipeline>();
        target.apply_rotation_pipeline->compute_shader = shader;

        target.apply_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = target.apply_rotation_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(DsBundle::RandomBundleApplySegmentsConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        target.apply_rotation_pipeline->Initialize();
      }

      {
        std::shared_ptr<Shader> shader;
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyPositionCorrections.slang");

        target.apply_position_pipeline = std::make_shared<ComputePipeline>();
        target.apply_position_pipeline->compute_shader = shader;

        target.apply_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = target.apply_position_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(DsBundle::RandomBundleApplySegmentsConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        target.apply_position_pipeline->Initialize();
      }

      {
        std::shared_ptr<Shader> shader;
        shader = std::make_shared<Shader>();
        shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                           std::filesystem::path("./EcoSysLabResources") /
                               "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyCorrections.slang");

        target.apply_position_rotation_pipeline = std::make_shared<ComputePipeline>();
        target.apply_position_rotation_pipeline->compute_shader = shader;

        target.apply_position_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = target.apply_position_rotation_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(DsBundle::RandomBundleApplySegmentsConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        target.apply_position_rotation_pipeline->Initialize();
      }

      {
        std::shared_ptr<Shader> shader;
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ConnectionCorrections.slang");

        target.connections_pipeline = std::make_shared<ComputePipeline>();
        target.connections_pipeline->compute_shader = shader;
        target.connections_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
        auto& stretch_shear_push_constant_range = target.connections_pipeline->push_constant_ranges.emplace_back();
        stretch_shear_push_constant_range.size = sizeof(DsBundle::RandomBundleApplyConnectionsConstant);
        stretch_shear_push_constant_range.offset = 0;
        stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        target.connections_pipeline->Initialize();
      }
    }
    ImGui::TreePop();
  }
  return changed;
}
bool eco_sys_lab_package::InspectDsFungus(InspectorContext& context, DsFungus& target) {
  bool changed = false;
  return changed;
}
bool eco_sys_lab_package::InspectDsPrediction(InspectorContext& context, DsPrediction& target) {
  bool changed = false;
  /*
  if (ImGui::DragFloat("Snow factor", &snow_factor, 1.f, 1.f, 100.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Snow deduction", &snow_deduction, .01f, .0f, 1.f)) {
    changed = true;
  }*/
  return changed;
}
bool eco_sys_lab_package::InspectDsDynamicHashedGrid(InspectorContext& context, DsDynamicHashedGrid& target) {
  bool changed = false;
  if (ImGui::DragFloat("Collision Range", &target.grid_cell_size, 0.001f, 0.001f, 1.0f))
    changed = true;
  return changed;
}
