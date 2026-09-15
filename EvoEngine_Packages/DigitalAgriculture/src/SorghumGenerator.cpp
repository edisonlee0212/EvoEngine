#include "Application.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "ProjectManager.hpp"
#include "SorghumLayer.hpp"

#include "Plot2D.hpp"
#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumSpline.hpp"
#include "Times.hpp"

using namespace digital_agriculture_package;

void digital_agriculture_package::SerializeSorghumGenerator(YAML::Emitter& out, const SorghumGenerator& target) {
  target.panicle_size.Save("panicle_size", out);
  target.panicle_seed_amount.Save("panicle_seed_amount", out);
  target.panicle_seed_radius.Save("panicle_seed_radius", out);

  target.stem_tilt_angle.Save("stem_tilt_angle", out);
  target.internode_length.Save("internode_length", out);
  target.stem_width.Save("stem_width", out);

  target.leaf_amount.Save("leaf_amount", out);
  target.leaf_starting_point.Save("leaf_starting_point", out);
  target.leaf_curling.Save("leaf_curling", out);

  target.leaf_roll_angle.Save("leaf_roll_angle", out);
  target.leaf_branching_angle.Save("leaf_branching_angle", out);

  target.leaf_bending.Save("leaf_bending", out);
  target.leaf_bending_acceleration.Save("leaf_bending_acceleration", out);
  target.leaf_bending_smoothness.Save("leaf_bending_smoothness", out);
  target.leaf_waviness.Save("leaf_waviness", out);
  target.leaf_waviness_frequency.Save("leaf_waviness_frequency", out);
  target.leaf_length.Save("leaf_length", out);
  target.leaf_width.Save("leaf_width", out);

  target.width_along_stem.Save("width_along_stem", out);
  target.width_along_leaf.Save("width_along_leaf", out);
  target.waviness_along_leaf.Save("waviness_along_leaf", out);
  target.curling_along_leaf.Save("curling_along_leaf", out);
}
void digital_agriculture_package::DeserializeSorghumGenerator(const YAML::Node& in, SorghumGenerator& target) {
  target.panicle_size.Load("panicle_size", in);
  target.panicle_seed_amount.Load("panicle_seed_amount", in);
  target.panicle_seed_radius.Load("panicle_seed_radius", in);

  target.stem_tilt_angle.Load("stem_tilt_angle", in);
  target.internode_length.Load("internode_length", in);
  target.stem_width.Load("stem_width", in);

  target.leaf_amount.Load("leaf_amount", in);
  target.leaf_starting_point.Load("leaf_starting_point", in);
  target.leaf_curling.Load("leaf_curling", in);

  target.leaf_roll_angle.Load("leaf_roll_angle", in);
  target.leaf_branching_angle.Load("leaf_branching_angle", in);

  target.leaf_bending.Load("leaf_bending", in);
  target.leaf_bending_acceleration.Load("leaf_bending_acceleration", in);
  target.leaf_bending_smoothness.Load("leaf_bending_smoothness", in);
  target.leaf_waviness.Load("leaf_waviness", in);
  target.leaf_waviness_frequency.Load("leaf_waviness_frequency", in);
  target.leaf_length.Load("leaf_length", in);
  target.leaf_width.Load("leaf_width", in);

  target.width_along_stem.Load("width_along_stem", in);
  target.width_along_leaf.Load("width_along_leaf", in);
  target.waviness_along_leaf.Load("waviness_along_leaf", in);
  target.curling_along_leaf.Load("curling_along_leaf", in);
}

Entity SorghumGenerator::CreateEntity(const unsigned int seed) const {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto entity = scene->CreateEntity(GetTitle());
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(entity).lock();
  const auto sorghum_state = AssetManager::CreateTemporaryAsset<SorghumState>();
  Apply(sorghum_state, seed);
  sorghum->sorghum_state = sorghum_state;
  sorghum->sorghum_generator = GetSelf();
  sorghum->GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  return entity;
}

void SorghumGenerator::Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor,
                             const unsigned int seed) const {
  const auto sorghum_state = AssetManager::CreateTemporaryAsset<SorghumState>();
  Apply(sorghum_state, seed);
  sorghum_state->Apply(target_sorghum_descriptor);
}

void SorghumGenerator::Apply(const std::shared_ptr<SorghumState>& target_sorghum_state, const unsigned seed) const {
  if (seed > 0)
    srand(seed);
  // Panicle
  target_sorghum_state->panicle.seed_amount = static_cast<int>(panicle_seed_amount.GetValue());
  const auto current_panicle_size = this->panicle_size.GetValue();
  target_sorghum_state->panicle.panicle_size =
      glm::vec3(current_panicle_size.x, current_panicle_size.y, current_panicle_size.x);
  target_sorghum_state->panicle.seed_radius = panicle_seed_radius.GetValue();
  // Stem
  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);

  target_sorghum_state->stem.direction = glm::normalize(glm::rotate(
      up_direction, glm::radians(glm::gaussRand(stem_tilt_angle.mean, stem_tilt_angle.deviation)), front_direction));
  const int leaf_size = static_cast<int>(glm::clamp(leaf_amount.GetValue(), 2.0f, 128.0f));
  target_sorghum_state->stem.length =
      internode_length.GetValue() * static_cast<float>(leaf_size) / (1.f - leaf_starting_point.GetValue(0));
  target_sorghum_state->stem.width_along_stem = {0.0f, stem_width.GetValue(), width_along_stem};
  // Leaves
  target_sorghum_state->leaves.resize(leaf_size);
  for (int leaf_index = 0; leaf_index < leaf_size; leaf_index++) {
    const float step = static_cast<float>(leaf_index) / (static_cast<float>(leaf_size) - 1.0f);
    auto& leaf_state = target_sorghum_state->leaves[leaf_index];
    leaf_state.index = leaf_index;
    leaf_state.starting_point = leaf_starting_point.GetValue(step);
    leaf_state.length = leaf_length.GetValue(step);
    if (leaf_state.length == 0.0f)
      continue;

    leaf_state.waviness_along_leaf = {0.0f, leaf_waviness.GetValue(step) * 2.0f, waviness_along_leaf};
    leaf_state.width_along_leaf = {0.0f, leaf_width.GetValue(step) * 2.0f, width_along_leaf};
    const auto curling = glm::clamp(leaf_curling.GetValue(step), 0.0f, 90.0f) / 90.0f;
    // leaf_state.curling_along_leaf = {0.0f, curling * 90.0f, curling_along_leaf};
    leaf_state.curling_along_leaf = {0.0f, curling * 90.0f, {1.f, 1.f}};
    leaf_state.branching_angle = leaf_branching_angle.GetValue(step);
    leaf_state.roll_angle = glm::mod((leaf_index % 2) * 180.0f + leaf_roll_angle.GetValue(step), 360.0f);
    auto bending = leaf_bending.GetValue(step);
    bending = (bending + 180) / 360.0f;
    const auto bending_acceleration = leaf_bending_acceleration.GetValue(step);
    const auto bending_smoothness = leaf_bending_smoothness.GetValue(step);

    leaf_state.bending_along_leaf = {-180.0f, 180.0f, {0.5f, bending}};
    const glm::vec2 middle = glm::mix(glm::vec2(0, bending), glm::vec2(1, 0.5f), bending_acceleration);
    auto& bending_along_leaf_curve = leaf_state.bending_along_leaf.curve.UnsafeGetValues();
    bending_along_leaf_curve.clear();
    bending_along_leaf_curve.emplace_back(-0.1, 0.0f);
    bending_along_leaf_curve.emplace_back(0, 0.5f);
    glm::vec2 left_delta = {middle.x, middle.y - 0.5f};
    bending_along_leaf_curve.push_back(left_delta * (1.0f - bending_smoothness));
    glm::vec2 right_delta = {middle.x - 1.0f, bending - middle.y};
    bending_along_leaf_curve.push_back(right_delta * (1.0f - bending_smoothness));
    bending_along_leaf_curve.emplace_back(1.0, bending);
    bending_along_leaf_curve.emplace_back(0.1, 0.0f);

    leaf_state.waviness_frequency = leaf_waviness_frequency.GetValue(step);
    leaf_state.waviness_period_start = glm::vec2(glm::linearRand(0.f, 100.f), glm::linearRand(0.f, 100.f));
  }
}

void SorghumGenerator::OnCreate() {
  panicle_size.mean = glm::vec3(0.0, 0.0, 0.0);
  panicle_seed_amount.mean = 0;
  panicle_seed_radius.mean = 0.002f;

  stem_tilt_angle.mean = 0.0f;
  stem_tilt_angle.deviation = 0.0f;
  internode_length.mean = 0.449999988f;
  internode_length.deviation = 0.150000006f;
  stem_width.mean = 0.0140000004f;
  stem_width.deviation = 0.0f;

  leaf_amount.mean = 9.0f;
  leaf_amount.deviation = 1.0f;

  leaf_starting_point.mean = {0.0f, 1.0f, Curve2D(0.1f, 1.0f)};
  leaf_starting_point.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};

  leaf_curling.mean = {0.0f, 90.0f, Curve2D(0.3f, 0.7f)};
  leaf_curling.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};
  leaf_roll_angle.mean = {-1.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_roll_angle.deviation = {0.0f, 6.0f, Curve2D(0.3f, 1.0f)};

  leaf_branching_angle.mean = {0.0f, 55.0f, Curve2D(0.5f, 0.2f)};
  leaf_branching_angle.deviation = {0.0f, 3.0f, Curve2D(0.67f, 0.225f)};

  leaf_bending.mean = {-180.0f, 180.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_bending_acceleration.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_smoothness.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_acceleration.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_waviness.mean = {0.0f, 20.0f, Curve2D(0.5f, 0.5f)};
  leaf_waviness.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_waviness_frequency.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_waviness_frequency.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_length.mean = {0.0f, 2.5f, Curve2D(0.165f, 0.247f)};
  leaf_length.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_width.mean = {0.0f, 0.075f, Curve2D(0.5f, 0.5f)};
  leaf_width.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  width_along_stem = Curve2D(1.0f, 0.1f);
  width_along_leaf = Curve2D(0.5f, 0.1f);
  waviness_along_leaf = Curve2D(0.0f, 0.5f);
}
