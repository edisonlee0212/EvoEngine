#include <gtest/gtest.h>
#include <glm/gtx/string_cast.hpp>

#include "Application.hpp"
#include "ComputePipeline.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "RenderLayer.hpp"
#include "RenderTexture.hpp"
#include "Shader.hpp"
#include "StarHover.hpp"
#include "TextureStorage.hpp"
#include "UniverseLayer.hpp"
#include "UniverseSerializationAdapters.hpp"

namespace evo_engine {
class PlatformLifecycleTestAccess {
 public:
  static void Initialize(const ApplicationInitializationSettings& settings) {
    Platform::Initialize(settings);
  }
  static void Destroy() {
    Platform::OnDestroy();
  }
};
}  // namespace evo_engine

using namespace universe_package;

namespace {
std::shared_ptr<StarCluster> Cluster(const uint32_t count, const uint64_t seed = 1) {
  auto cluster = std::make_shared<StarCluster>();
  cluster->SetStarCount(count);
  cluster->seed = seed;
  return cluster;
}

void ExpectSample(const StarBaseSample& a, const StarBaseSample& b) {
  EXPECT_EQ(0, std::memcmp(&a, &b, sizeof(a)));
}
}  // namespace

TEST(UniverseStarCluster, DeterministicSamplesDependOnlyOnSeedAndOrdinal) {
  const auto first = GenerateStarBaseSample(42, 17);
  ExpectSample(first, GenerateStarBaseSample(42, 17));
  const auto reseeded = GenerateStarBaseSample(43, 17);
  EXPECT_NE(0, std::memcmp(&first, &reseeded, sizeof(first)));
}

TEST(UniverseStarCluster, GrowthShrinkAndRepackPreserveSamples) {
  const auto first = Cluster(3);
  const auto second = Cluster(2, 9);
  StarClusterBatch batch;
  ASSERT_TRUE(batch.Update({{first}, {second}}, 0));
  const auto original = batch.samples;
  first->SetStarCount(5);
  ASSERT_TRUE(batch.Update({{first}, {second}}, 1));
  EXPECT_EQ(5u, batch.ranges[1].offset);
  for (uint32_t i = 0; i < 3; ++i)
    ExpectSample(original[i], batch.samples[i]);
  ExpectSample(original[3], batch.samples[5]);
  first->SetStarCount(1);
  batch.Update({{first}, {second}}, 2);
  EXPECT_EQ(1u, batch.ranges[1].offset);
  ExpectSample(original[3], batch.samples[1]);
  first->SetStarCount(0);
  batch.Update({{first}, {second}}, 3);
  ASSERT_EQ(1u, batch.ranges.size());
  EXPECT_EQ(0u, batch.ranges[0].offset);
  ExpectSample(original[3], batch.samples[0]);
}

TEST(UniverseStarCluster, RadiusSamplesAreNormalAndSizesAreClamped) {
  StarCluster cluster;
  cluster.radius_standard_deviation = 0.2;
  cluster.radius_min = 0.1;
  cluster.radius_max = 2;
  auto p = BuildStarClusterParameters(cluster, glm::dmat4(1), 0);
  double sum = 0, square_sum = 0, cross_sum = 0;
  constexpr uint32_t count = 50000;
  for (uint32_t i = 0; i < count; ++i) {
    const auto sample = GenerateStarBaseSample(42, i);
    sum += sample.gaussian_radius;
    square_sum += sample.gaussian_radius * sample.gaussian_radius;
    cross_sum += sample.gaussian_radius * sample.gaussian_z;
    const double radius = EvaluateStar(p, sample).world_position_radius.w;
    EXPECT_GE(radius, cluster.radius_min);
    EXPECT_LE(radius, cluster.radius_max);
    EXPECT_DOUBLE_EQ(radius, glm::clamp(1 + 0.2 * sample.gaussian_radius, 0.1, 2.0));
  }
  EXPECT_NEAR(sum / count, 0, 0.02);
  EXPECT_NEAR(square_sum / count, 1, 0.03);
  EXPECT_NEAR(cross_sum / count, 0, 0.02);
  auto sample = GenerateStarBaseSample(42, 0);
  sample.gaussian_radius = -100;
  EXPECT_DOUBLE_EQ(EvaluateStar(p, sample).world_position_radius.w, 0.1);
  sample.gaussian_radius = 100;
  EXPECT_DOUBLE_EQ(EvaluateStar(p, sample).world_position_radius.w, 2);
  cluster.radius_min = cluster.radius_max = 0.5;
  p = BuildStarClusterParameters(cluster, glm::dmat4(1), 0);
  EXPECT_DOUBLE_EQ(EvaluateStar(p, sample).world_position_radius.w, 0.5);
  cluster.radius_min = -1;
  cluster.radius_max = -2;
  p = BuildStarClusterParameters(cluster, glm::dmat4(1), 0);
  EXPECT_DOUBLE_EQ(EvaluateStar(p, sample).world_position_radius.w, 0);
  cluster.radius_standard_deviation = 0;
  p = BuildStarClusterParameters(cluster, glm::dmat4(1), 0);
  EXPECT_DOUBLE_EQ(EvaluateStar(p, sample).world_position_radius.w, cluster.visual_radius);
}

TEST(UniverseStarCluster, RadiusEditsOnlyUpdateParametersAndDoNotMoveStars) {
  const auto first = Cluster(3);
  const auto second = Cluster(2, 9);
  StarClusterBatch batch;
  batch.Update({{first}, {second}}, 0);
  const auto samples = batch.samples;
  const auto original = EvaluateStar(batch.parameters[0], batch.samples[0]);
  const auto other = batch.parameters[1];
  const auto revision = batch.population_revision;
  first->visual_radius = 2;
  first->radius_standard_deviation = 0.5;
  first->radius_min = 1.5;
  first->radius_max = 3;
  EXPECT_FALSE(batch.Update({{first}, {second}}, 0));
  EXPECT_EQ(batch.population_revision, revision);
  for (size_t i = 0; i < samples.size(); ++i)
    ExpectSample(samples[i], batch.samples[i]);
  EXPECT_EQ(std::memcmp(&other, &batch.parameters[1], sizeof(other)), 0);
  const auto updated = EvaluateStar(batch.parameters[0], batch.samples[0]);
  EXPECT_EQ(glm::dvec3(original.world_position_radius), glm::dvec3(updated.world_position_radius));
  EXPECT_GE(updated.world_position_radius.w, 1.5);
  EXPECT_LE(updated.world_position_radius.w, 3);
  EXPECT_EQ(original.color_emission, updated.color_emission);
}

TEST(UniverseStarCluster, SettingsAndTransformsDoNotRebuildSamples) {
  const auto first = Cluster(8);
  StarClusterBatch batch;
  batch.Update({{first}}, 0);
  const auto* storage = batch.samples.data();
  const auto revision = batch.population_revision;
  first->disk_diameter = 900;
  first->disk_color = glm::vec3(0.25f);
  glm::dmat4 transform(1.0);
  transform[3] = glm::dvec4(10, 20, 30, 1);
  EXPECT_FALSE(batch.Update({{first, transform}}, 1));
  EXPECT_EQ(storage, batch.samples.data());
  EXPECT_EQ(revision, batch.population_revision);
  EXPECT_EQ(900, batch.parameters[0].ellipse0.x * 2);
  EXPECT_EQ(transform[3], batch.parameters[0].world3);
  EXPECT_EQ(0.25f, batch.parameters[0].disk_color_intensity.x);
}

TEST(UniverseStarCluster, SeedEditChangesOnlyItsCluster) {
  const auto first = Cluster(3);
  const auto second = Cluster(2, 9);
  StarClusterBatch batch;
  batch.Update({{first}, {second}}, 0);
  const auto original = batch.samples;
  second->seed = 77;
  EXPECT_TRUE(batch.Update({{first}, {second}}, 1));
  ExpectSample(original[0], batch.samples[0]);
  ExpectSample(GenerateStarBaseSample(77, 0), batch.samples[3]);
}

TEST(UniverseStarCluster, IndependentClocksSurvivePauseDisableAndRepack) {
  const auto first = Cluster(1);
  const auto second = Cluster(1);
  first->phase = second->phase = 0;
  first->time_scale = 2;
  second->time_scale = -3;
  StarClusterBatch batch;
  batch.Update({{first}, {second}}, 10);
  batch.Update({{first}, {second}}, 11);
  EXPECT_DOUBLE_EQ(2, batch.parameters[0].time_padding.x);
  EXPECT_DOUBLE_EQ(-3, batch.parameters[1].time_padding.x);
  first->paused = true;
  batch.Update({{first}, {second, glm::dmat4(1), false}}, 12);
  EXPECT_DOUBLE_EQ(2, batch.parameters[0].time_padding.x);
  first->paused = false;
  first->SetStarCount(257);
  batch.Update({{first}, {second}}, 13);
  EXPECT_DOUBLE_EQ(4, batch.parameters[0].time_padding.x);
  EXPECT_DOUBLE_EQ(-9, batch.parameters[1].time_padding.x);
  EXPECT_EQ(257u, batch.ranges[1].offset);
  EXPECT_EQ(2u, batch.clocks.size());
}

TEST(UniverseStarCluster, DeletionAndSceneResetDropState) {
  const auto first = Cluster(255);
  const auto second = Cluster(256);
  StarClusterBatch batch;
  batch.Update({{first}, {second}}, 0);
  EXPECT_TRUE(batch.Update({{second}}, 1));
  EXPECT_EQ(1u, batch.clocks.size());
  EXPECT_EQ(batch.clocks.at(second.get()).identity, batch.ranges[0].identity);
  EXPECT_EQ(0u, batch.ranges[0].offset);
  batch = {};
  batch.Update({{second}}, 10);
  EXPECT_DOUBLE_EQ(second->phase, batch.parameters[0].time_padding.x);
  EXPECT_TRUE(batch.Update({}, 11));
  EXPECT_TRUE(batch.samples.empty());
  EXPECT_TRUE(batch.parameters.empty());
  EXPECT_TRUE(batch.clocks.empty());
}

TEST(UniverseStarCluster, PopulationBoundaryCountsAndTwoLargeClusters) {
  StarClusterBatch batch;
  const auto first = Cluster(0);
  const auto second = Cluster(0, 2);
  for (const auto count : {0u, 1u, 255u, 256u, 257u, 250000u}) {
    first->SetStarCount(count);
    second->SetStarCount(count);
    batch.Update({{first}, {second}}, 0);
    EXPECT_EQ(static_cast<size_t>(count) * 2, batch.samples.size());
    if (count != 0) {
      EXPECT_EQ(count, batch.ranges[1].offset);
      EXPECT_EQ(count, batch.parameters[1].star_count);
      ExpectSample(GenerateStarBaseSample(2, count - 1), batch.samples.back());
    }
  }
}

TEST(UniverseStarCluster, SerializationPersistsOnlyAuthoringAndClonesIndependently) {
  const auto source = Cluster(3, 77);
  source->disk_diameter = 456;
  source->time_scale = -2.5;
  source->paused = true;
  source->visual_radius = 1.5;
  source->radius_standard_deviation = 0.4;
  source->radius_min = 0.2;
  source->radius_max = 3;
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  SerializeStarCluster(emitter, *source);
  emitter << YAML::EndMap;
  const std::string text = emitter.c_str();
  for (const auto* obsolete : {"capacity", "buffer", "revision", "particle", "star_ids", "next_id", "depth_write"})
    EXPECT_EQ(std::string::npos, text.find(obsolete));
  const auto clone = Cluster(0);
  DeserializeStarCluster(YAML::Load(text), *clone);
  clone->PostCloneAction(source);
  EXPECT_EQ(3u, clone->GetStarCount());
  EXPECT_EQ(77u, clone->seed);
  EXPECT_EQ(456, clone->disk_diameter);
  EXPECT_EQ(-2.5, clone->time_scale);
  EXPECT_TRUE(clone->paused);
  EXPECT_EQ(1.5, clone->visual_radius);
  EXPECT_EQ(0.4, clone->radius_standard_deviation);
  EXPECT_EQ(0.2, clone->radius_min);
  EXPECT_EQ(3, clone->radius_max);
  clone->radius_standard_deviation = 0.8;
  EXPECT_EQ(0.4, source->radius_standard_deviation);
  clone->SetStarCount(1);
  EXPECT_EQ(3u, source->GetStarCount());
  StarClusterBatch batch;
  batch.Update({{source}, {clone}}, 0);
  EXPECT_EQ(2u, batch.clocks.size());
  ExpectSample(batch.samples[0], batch.samples[3]);
}

TEST(UniverseStarCluster, LegacyIdsMigrateToCountAndExplicitCountWins) {
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  evo_engine::Serialization::SerializeVector("star_ids", std::vector<uint64_t>{900, 2, 77}, emitter);
  emitter << YAML::Key << "next_id_" << YAML::Value << 901;
  emitter << YAML::Key << "depth_write" << YAML::Value << false;
  emitter << YAML::EndMap;
  auto node = YAML::Load(emitter.c_str());
  StarCluster restored;
  DeserializeStarCluster(node, restored);
  EXPECT_EQ(3u, restored.GetStarCount());
  node["star_count"] = 0;
  DeserializeStarCluster(node, restored);
  EXPECT_EQ(0u, restored.GetStarCount());
  node["visual_radius"] = 7;
  DeserializeStarCluster(node, restored);
  EXPECT_EQ(0, restored.radius_standard_deviation);
  EXPECT_EQ(7, EvaluateStar(BuildStarClusterParameters(restored, glm::dmat4(1), 0), GenerateStarBaseSample(1, 0))
                   .world_position_radius.w);
}

TEST(UniverseStarCluster, LiveClonesWithCopiedHandlesKeepIndependentClocksAndRanges) {
  const auto first = Cluster(3, 7);
  const auto clone = std::make_shared<StarCluster>(*first);
  ASSERT_EQ(first->GetHandle().GetValue(), clone->GetHandle().GetValue());
  first->time_scale = 2;
  clone->time_scale = -3;
  StarClusterBatch batch;
  batch.Update({{first}, {clone}}, 10);
  batch.Update({{first}, {clone}}, 11);
  EXPECT_EQ(2u, batch.clocks.size());
  EXPECT_NE(batch.ranges[0].identity, batch.ranges[1].identity);
  EXPECT_DOUBLE_EQ(first->phase + 2, batch.parameters[0].time_padding.x);
  EXPECT_DOUBLE_EQ(clone->phase - 3, batch.parameters[1].time_padding.x);
  clone->seed = 9;
  first->SetStarCount(1);
  batch.Update({{clone}, {first}}, 12);
  EXPECT_EQ(3u, batch.ranges[1].offset);
  ExpectSample(GenerateStarBaseSample(9, 0), batch.samples[0]);
  ExpectSample(GenerateStarBaseSample(7, 0), batch.samples[3]);
}

TEST(UniverseStarCluster, PooledReuseRestoresAuthoringDefaults) {
  const auto cluster = Cluster(10, 99);
  cluster->disk_diameter = 12;
  cluster->paused = true;
  cluster->radius_standard_deviation = 3;
  cluster->radius_min = 2;
  cluster->radius_max = 5;
  cluster->OnDestroy();
  cluster->OnCreate();
  EXPECT_EQ(1u, cluster->seed);
  EXPECT_EQ(30000, cluster->disk_diameter);
  EXPECT_FALSE(cluster->paused);
  EXPECT_EQ(0u, cluster->GetStarCount());
  EXPECT_EQ(0, cluster->radius_standard_deviation);
  EXPECT_EQ(0.1, cluster->radius_min);
  EXPECT_EQ(2, cluster->radius_max);
}

TEST(UniverseStarCluster, GlobalDepthWriteDefaultsOnAndDoesNotChangePopulation) {
  UniverseLayer layer;
  EXPECT_TRUE(layer.depth_write);
  EXPECT_FLOAT_EQ(1.0f, layer.star_fade_strength);
  layer.depth_write = false;
  const auto cluster = Cluster(1);
  StarClusterBatch batch;
  batch.Update({{cluster}}, 0);
  const auto revision = batch.population_revision;
  layer.depth_write = true;
  layer.star_fade_strength = 2.0f;
  EXPECT_FALSE(batch.Update({{cluster}}, 1));
  EXPECT_EQ(revision, batch.population_revision);
}

TEST(UniverseStarCluster, GpuBufferLayoutsRemainExact) {
  EXPECT_EQ(40u, sizeof(StarBaseSample));
  EXPECT_EQ(32u, offsetof(StarBaseSample, gaussian_radius));
  EXPECT_EQ(448u, sizeof(StarClusterGpuParameters));
  EXPECT_EQ(64u, sizeof(StarClusterGpuResult));
  EXPECT_EQ(8u, sizeof(StarClusterComputePushConstant));
  EXPECT_EQ(4u, offsetof(StarClusterComputePushConstant, star_offset));
  EXPECT_EQ(0u, offsetof(StarClusterGpuResult, world_position_radius));
  EXPECT_EQ(32u, offsetof(StarClusterGpuResult, color_emission));
  EXPECT_EQ(48u, offsetof(StarClusterGpuResult, alpha_padding));
  EXPECT_EQ(0u, offsetof(StarClusterGpuParameters, population_revision));
  EXPECT_EQ(16u, offsetof(StarClusterGpuParameters, ellipse0));
  EXPECT_EQ(240u, offsetof(StarClusterGpuParameters, world0));
  EXPECT_EQ(368u, offsetof(StarClusterGpuParameters, disk_color_intensity));
  EXPECT_EQ(416u, offsetof(StarClusterGpuParameters, time_padding));
}

namespace {
class UniverseGpuFixture {
 public:
  UniverseGpuFixture() {
    application_ = std::make_unique<evo_engine::Application>();
    application_->PushLayer<RenderLayer>("Render Layer");
    Jobs::Initialize(2);
    ApplicationInitializationSettings settings;
    settings.load_default_resources = false;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    settings.enable_runtime_packages = false;
    settings.graphics_settings.use_mesh_shader = false;
    settings.graphics_settings.use_ray_tracing = false;
    const_cast<ApplicationInitializationSettings&>(application_->GetApplicationInfo()) = settings;
    evo_engine::PlatformLifecycleTestAccess::Initialize(settings);
  }
  ~UniverseGpuFixture() {
    application_->PopLayer<RenderLayer>();
    Platform::DrainGpuResourceWork();
    TextureStorage::OnDestroy();
    GeometryStorage::OnDestroy();
    evo_engine::PlatformLifecycleTestAccess::Destroy();
    Jobs::OnDestroy();
  }

 private:
  std::unique_ptr<evo_engine::Application> application_;
};

std::shared_ptr<Buffer> TestBuffer(const size_t size) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = size;
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<Buffer>(info, allocation);
}

StarClusterGpuResult ReferenceStar(const StarClusterGpuParameters& p, const StarBaseSample& sample) {
  const double proportion = sample.orbital_proportion;
  const bool disk = proportion > p.ellipse1.z;
  const double t = disk ? (proportion - p.ellipse1.z) / (1 - p.ellipse1.z) : proportion / p.ellipse1.z;
  const auto mix = [t](const double a, const double b) {
    return a + (b - a) * t;
  };
  const double a = disk ? mix(p.ellipse0.z, p.ellipse0.x) : mix(p.ellipse1.x, p.ellipse0.z);
  const double b = disk ? mix(p.ellipse0.w, p.ellipse0.y) : mix(p.ellipse1.y, p.ellipse0.w);
  const double tilt_x = disk ? mix(p.speed_tilt.w, p.speed_tilt.y) : mix(p.tilt_radius.y, p.speed_tilt.w);
  const double tilt_z = disk ? mix(p.tilt_radius.x, p.speed_tilt.z) : mix(p.tilt_radius.z, p.tilt_radius.x);
  const double speed = disk ? mix(p.spread_speed.w, p.spread_speed.z) : mix(p.speed_tilt.x, p.spread_speed.w);
  const double angle = (proportion * 360 + p.time_padding.x) / std::sqrt(a + b) * speed;
  glm::dvec3 point(std::sin(angle) * a, 0, std::cos(angle) * b);
  const auto rotate = [&](const int axis, const double degrees) {
    const double s = std::sin(glm::radians(degrees)), c = std::cos(glm::radians(degrees));
    if (axis == 0)
      point = {point.x, c * point.y - s * point.z, s * point.y + c * point.z};
    else if (axis == 1)
      point = {c * point.x + s * point.z, point.y, -s * point.x + c * point.z};
    else
      point = {c * point.x - s * point.y, s * point.x + c * point.y, point.z};
  };
  rotate(0, tilt_x);
  rotate(1, -p.ellipse1.w * proportion);
  rotate(2, tilt_z);
  point += glm::dvec3(p.center_offset) * (1 - proportion) + glm::dvec3(p.center_position);
  point += glm::dvec3(sample.gaussian_x * p.spread_speed.y, sample.gaussian_y * p.spread_speed.x,
                      sample.gaussian_z * p.spread_speed.y) *
           (p.ellipse0.x + p.ellipse0.y);
  point /= 20;
  const glm::dvec4 world = glm::dmat4(p.world0, p.world1, p.world2, p.world3) * glm::dvec4(point, 1);
  StarClusterGpuResult result;
  result.world_position_radius = glm::dvec4(glm::dvec3(world), p.tilt_radius.w);
  if (p.center_offset.w > 0)
    result.world_position_radius.w =
        glm::clamp(p.tilt_radius.w + p.center_offset.w * sample.gaussian_radius, p.time_padding.z, p.time_padding.w);
  result.color_emission = disk ? glm::mix(p.core_color_intensity, p.disk_color_intensity, float(t))
                               : glm::mix(p.center_color_intensity, p.core_color_intensity, float(t));
  result.alpha_padding.x = static_cast<float>(p.time_padding.y);
  return result;
}
}  // namespace

TEST(UniverseStarCluster, GpuPackedRangesMatchDoublePrecisionReference) {
  UniverseGpuFixture gpu;
  if (!Platform::GetSelectedPhysicalDevice()->features.shaderFloat64)
    GTEST_SKIP() << "shaderFloat64 unavailable";
  auto layout = std::make_shared<DescriptorSetLayout>();
  for (uint32_t binding = 0; binding < 3; ++binding)
    layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  auto shader = std::make_shared<Shader>();
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  ASSERT_TRUE(shader->TryCompile(
      ShaderType::Compute, Platform::GetShaderGlobalDefines(),
      root / "EvoEngine_Packages/Universe/Internals/UniverseResources/Shaders/Compute/StarCluster.slang"));
  auto pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = shader;
  pipeline->descriptor_set_layouts.push_back(layout);
  pipeline->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(StarClusterComputePushConstant)});
  pipeline->Initialize();
  ASSERT_TRUE(pipeline->Initialized());

  const auto first = Cluster(0);
  const auto second = Cluster(0, 2);
  first->disk_tilt_x = 27;
  first->core_tilt_z = -13;
  first->radius_standard_deviation = 0.3;
  first->radius_min = 0.2;
  first->radius_max = 1.7;
  second->disk_color = glm::vec3(1, 0.15, 0.05);
  second->phase = -3000;
  second->visual_radius = 0.7;
  second->radius_standard_deviation = 0.5;
  second->radius_min = 0.4;
  second->radius_max = 2;
  second->alpha = 0.4f;
  auto first_world = glm::rotate(glm::dmat4(1), glm::radians(-13.0), glm::dvec3(0, 1, 0));
  first_world[3] = glm::dvec4(-500, 80, 30, 1);
  auto world = glm::rotate(glm::dmat4(1), glm::radians(31.0), glm::normalize(glm::dvec3(1, 2, 3)));
  world[0] *= 1.5;
  world[3] = glm::dvec4(180, 20, -80, 1);
  StarClusterBatch batch;
  for (const auto counts :
       {glm::uvec2(0, 0), glm::uvec2(1, 0), glm::uvec2(255, 257), glm::uvec2(256, 1), glm::uvec2(257, 255),
        glm::uvec2(250000, 0), glm::uvec2(250000, 250000), glm::uvec2(500000, 0)}) {
    first->SetStarCount(counts.x);
    second->SetStarCount(counts.y);
    batch.Update({{first, first_world}, {second, world}}, 1, counts.x == 500000 ? 100 : 1);
    const size_t count = batch.samples.size();
    if (count == 0) {
      EXPECT_TRUE(batch.ranges.empty());
      continue;
    }
    auto parameters = TestBuffer(batch.parameters.size() * sizeof(StarClusterGpuParameters));
    auto samples = TestBuffer(count * sizeof(StarBaseSample));
    auto output = TestBuffer((count + 1) * sizeof(StarClusterGpuResult));
    parameters->UploadVector(batch.parameters);
    samples->UploadVector(batch.samples);
    std::vector<StarClusterGpuResult> actual(count + 1);
    actual.back().world_position_radius = glm::dvec4(-12345);
    output->UploadVector(actual);
    auto descriptor = std::make_shared<DescriptorSet>(layout);
    descriptor->UpdateBufferDescriptorBinding(0, parameters);
    descriptor->UpdateBufferDescriptorBinding(1, samples);
    descriptor->UpdateBufferDescriptorBinding(2, output);
    const auto dispatch = [&] {
      Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
        pipeline->Bind(command);
        pipeline->BindDescriptorSet(command, 0, descriptor->GetVkDescriptorSet());
        for (uint32_t i = 0; i < batch.ranges.size(); ++i) {
          pipeline->PushConstant(command, 0, StarClusterComputePushConstant{i, batch.ranges[i].offset});
          pipeline->Dispatch(command, Platform::DivUp(batch.ranges[i].count, 256u));
        }
        Platform::BufferMemoryBarrier(command, *output, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                      VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                      VK_ACCESS_2_TRANSFER_READ_BIT);
      });
      output->DownloadData(actual.size() * sizeof(StarClusterGpuResult), actual.data());
    };
    dispatch();
    EXPECT_EQ(glm::dvec4(-12345), actual.back().world_position_radius);
    double max_position_error = 0;
    float max_color_error = 0;
    size_t max_index = 0;
    for (size_t r = 0; r < batch.ranges.size(); ++r) {
      const auto& range = batch.ranges[r];
      for (uint32_t i = 0; i < range.count; ++i) {
        const auto index = range.offset + i;
        const auto reference = ReferenceStar(batch.parameters[r], batch.samples[index]);
        const auto cpu = EvaluateStar(batch.parameters[r], batch.samples[index]);
        EXPECT_EQ(reference.world_position_radius, cpu.world_position_radius);
        EXPECT_EQ(reference.color_emission, cpu.color_emission);
        EXPECT_EQ(reference.alpha_padding.x, cpu.alpha_padding.x);
        for (int channel = 0; channel < 4; ++channel) {
          ASSERT_TRUE(std::isfinite(actual[index].world_position_radius[channel]));
          ASSERT_TRUE(std::isfinite(actual[index].color_emission[channel]));
          const auto error =
              std::abs(actual[index].world_position_radius[channel] - reference.world_position_radius[channel]);
          if (error > max_position_error) {
            max_position_error = error;
            max_index = index;
          }
          max_color_error = (std::max)(max_color_error, std::abs(actual[index].color_emission[channel] -
                                                                 reference.color_emission[channel]));
        }
        ASSERT_FLOAT_EQ(reference.alpha_padding.x, actual[index].alpha_padding.x);
      }
    }
    EXPECT_LT(max_position_error, 3e-5)
        << counts.x << ", " << counts.y << " index " << max_index << " actual "
        << glm::to_string(actual[max_index].world_position_radius) << " expected "
        << glm::to_string(
               ReferenceStar(batch.parameters[max_index >= counts.x && counts.y != 0 ? batch.parameters.size() - 1 : 0],
                             batch.samples[max_index])
                   .world_position_radius);
    EXPECT_LT(max_color_error, 1e-5f) << counts.x << ", " << counts.y;

    const auto world_results = actual;
    const auto selected = EvaluateStar(batch.parameters[0], batch.samples[0]);
    const auto world_to_star =
        glm::inverse(StarReferenceFrame(glm::dvec3(selected.world_position_radius), first_world));
    for (auto& p : batch.parameters) {
      const auto rebased = world_to_star * glm::dmat4(p.world0, p.world1, p.world2, p.world3);
      p.world0 = rebased[0];
      p.world1 = rebased[1];
      p.world2 = rebased[2];
      p.world3 = rebased[3];
    }
    parameters->UploadVector(batch.parameters);
    dispatch();
    EXPECT_EQ(glm::dvec4(-12345), actual.back().world_position_radius);
    EXPECT_LT(glm::length(glm::dvec3(actual[0].world_position_radius)), 3e-5);
    double maximum_rebase_error = 0;
    for (size_t r = 0; r < batch.ranges.size(); ++r) {
      const auto& range = batch.ranges[r];
      for (uint32_t i = 0; i < range.count; ++i) {
        const auto index = range.offset + i;
        const auto expected = world_to_star * glm::dvec4(glm::dvec3(world_results[index].world_position_radius), 1);
        const auto cpu = EvaluateStar(batch.parameters[r], batch.samples[index]);
        for (int channel = 0; channel < 3; ++channel) {
          maximum_rebase_error = (std::max)(maximum_rebase_error,
                                            std::abs(actual[index].world_position_radius[channel] - expected[channel]));
          maximum_rebase_error =
              (std::max)(maximum_rebase_error,
                         std::abs(actual[index].world_position_radius[channel] - cpu.world_position_radius[channel]));
        }
        EXPECT_EQ(world_results[index].world_position_radius.w, actual[index].world_position_radius.w);
        EXPECT_EQ(world_results[index].color_emission, actual[index].color_emission);
      }
    }
    EXPECT_LT(maximum_rebase_error, 6e-5) << counts.x << ", " << counts.y;
  }
}

TEST(UniverseStarCluster, OpaqueDiscsOccludeInBothOrdersWithoutShaderHalo) {
  UniverseGpuFixture gpu;
  auto vertex = std::make_shared<Shader>();
  ASSERT_TRUE(vertex->TryCompile(ShaderType::Vertex, std::string(R"(
struct Push { uint nearFirst; };
[[vk::push_constant]] ConstantBuffer<Push> constants;
struct Output {
  float4 position : SV_Position;
  [[vk::location(0)]] float2 discPosition : TEXCOORD0;
  [[vk::location(1)]] float3 radiance : COLOR0;
};
[shader("vertex")]
Output main(uint vertex : SV_VertexID, uint instance : SV_InstanceID) {
  const float2 corners[6] = {
    float2(-1,-1), float2(1,-1), float2(1,1),
    float2(-1,-1), float2(1,1), float2(-1,1)
  };
  bool nearStar = (instance == 0) == (constants.nearFirst != 0);
  Output result;
  result.discPosition = corners[vertex];
  result.position = float4(corners[vertex] * 0.8, nearStar ? 0.25 : 0.75, 1);
  result.radiance = nearStar ? float3(0,3,0) : float3(4,0,0);
  return result;
})")));
  auto fragment = std::make_shared<Shader>();
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  ASSERT_TRUE(fragment->TryCompile(
      ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
      root / "EvoEngine_Packages/Universe/Internals/UniverseResources/Shaders/Graphics/StarCluster.frag.slang"));
  GraphicsPipeline pipeline;
  pipeline.view_mask = 0;
  pipeline.vertex_shader = vertex;
  pipeline.fragment_shader = fragment;
  pipeline.geometry_type = GeometryType::Mesh;
  pipeline.vertex_input_enabled = false;
  pipeline.primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  pipeline.depth_attachment_format = Platform::Constants::render_texture_depth;
  pipeline.stencil_attachment_format = VK_FORMAT_UNDEFINED;
  pipeline.color_attachment_formats = {VK_FORMAT_R32G32B32A32_SFLOAT};
  pipeline.push_constant_ranges.push_back({VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(uint32_t)});
  pipeline.Initialize();
  ASSERT_TRUE(pipeline.Initialized());
  RenderTextureCreateInfo info;
  info.extent = {32, 32, 1};
  info.color_format = VK_FORMAT_R32G32B32A32_SFLOAT;
  RenderTexture target(info);
  const auto draw = [&](const uint32_t near_first, const bool depth_write, const float clear_depth) {
    ConfigureStarRenderStates(pipeline, {0, 0, 32, 32}, depth_write);
    EXPECT_FALSE(pipeline.states.color_blend_attachment_states[0].blendEnable);
    EXPECT_TRUE(pipeline.states.depth_test);
    EXPECT_EQ(depth_write, pipeline.states.depth_write);
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      target.GetColorImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      target.GetDepthImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      std::vector<VkRenderingAttachmentInfo> colors;
      target.AppendColorAttachmentInfos(colors, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      auto depth = target.GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      depth.clearValue.depthStencil.depth = clear_depth;
      VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
      rendering.renderArea = {{0, 0}, {32, 32}};
      rendering.layerCount = 1;
      rendering.colorAttachmentCount = 1;
      rendering.pColorAttachments = colors.data();
      rendering.pDepthAttachment = &depth;
      Platform::BeginRendering(command, rendering);
      pipeline.states.ApplyAllStates(command);
      pipeline.Bind(command);
      pipeline.PushConstant(command, 0, near_first);
      Platform::Draw(command, 6, 2);
      Platform::EndRendering(command);
      target.GetColorImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    });
    std::vector<glm::vec4> pixels;
    target.GetRgbaChannelData(pixels);
    return pixels;
  };
  for (uint32_t near_first : {0u, 1u}) {
    const auto pixels = draw(near_first, true, 1);
    EXPECT_EQ(glm::vec4(0, 3, 0, 1), pixels[16 * 32 + 16]);
    EXPECT_EQ(glm::vec4(0, 3, 0, 1), pixels[16 * 32 + 28]);  // Solid near the rim.
    EXPECT_EQ(glm::vec4(0), pixels[5 * 32 + 5]);             // Quad corner outside the disc is discarded.
    EXPECT_EQ(glm::vec4(0), pixels[0]);
  }
  EXPECT_EQ(glm::vec4(4, 0, 0, 1), draw(1, false, 1)[16 * 32 + 16]);
  EXPECT_EQ(glm::vec4(0, 3, 0, 1), draw(0, false, 1)[16 * 32 + 16]);
  EXPECT_EQ(glm::vec4(0), draw(0, true, 0.1f)[16 * 32 + 16]);  // Opaque scene depth occludes both.
}

TEST(UniverseStarCluster, MinimumPixelStarsFadeWithDistanceAndStayBelowBloom) {
  UniverseGpuFixture gpu;
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  Shader::RegisterShaderIncludePath(root / "EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules");
  const auto shaders = root / "EvoEngine_Packages/Universe/Internals/UniverseResources/Shaders/Graphics";
  auto vertex = std::make_shared<Shader>();
  auto fragment = std::make_shared<Shader>();
  ASSERT_TRUE(
      vertex->TryCompile(ShaderType::Vertex, Platform::GetShaderGlobalDefines(), shaders / "StarCluster.vert.slang"));
  ASSERT_TRUE(fragment->TryCompile(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                   shaders / "StarCluster.frag.slang"));
  auto camera_layout = std::make_shared<DescriptorSetLayout>();
  camera_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
  camera_layout->Initialize();
  auto star_layout = std::make_shared<DescriptorSetLayout>();
  star_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
  star_layout->Initialize();
  auto cameras_buffer = TestBuffer(2 * sizeof(CameraInfoBlock));
  auto stars_buffer = TestBuffer(sizeof(StarClusterGpuResult));
  auto camera_descriptor = std::make_shared<DescriptorSet>(camera_layout);
  auto star_descriptor = std::make_shared<DescriptorSet>(star_layout);
  camera_descriptor->UpdateBufferDescriptorBinding(2, cameras_buffer);
  star_descriptor->UpdateBufferDescriptorBinding(0, stars_buffer);
  struct Push {
    int32_t camera_index = 0;
    float brightness_limit = 0.8991f;
    glm::vec2 viewport_size = glm::vec2(33);
    float fade_strength = 1.0f;
  };
  static_assert(sizeof(Push) == 20);
  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Vertex, vertex->PeekShaderCode(), reflection, diagnostics,
                                   shaders / "StarCluster.vert.slang"))
      << diagnostics;
  ASSERT_EQ(reflection.push_constant_ranges.size(), 1u);
  EXPECT_EQ(reflection.push_constant_ranges[0].size, sizeof(Push));
  GraphicsPipeline pipeline;
  pipeline.view_mask = 0;
  pipeline.vertex_shader = vertex;
  pipeline.fragment_shader = fragment;
  pipeline.geometry_type = GeometryType::Mesh;
  pipeline.vertex_input_enabled = false;
  pipeline.primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  pipeline.depth_attachment_format = Platform::Constants::render_texture_depth;
  pipeline.stencil_attachment_format = VK_FORMAT_UNDEFINED;
  pipeline.color_attachment_formats = {VK_FORMAT_R32G32B32A32_SFLOAT};
  pipeline.descriptor_set_layouts = {camera_layout, star_layout};
  pipeline.push_constant_ranges.push_back({VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(Push)});
  pipeline.Initialize();
  ASSERT_TRUE(pipeline.Initialized());
  RenderTextureCreateInfo info;
  info.extent = {65, 65, 1};
  info.color_format = VK_FORMAT_R32G32B32A32_SFLOAT;
  RenderTexture target(info);
  std::vector<glm::vec4> last_pixels;
  const auto draw = [&](const bool perspective, const double radius, const double distance, const Push& push,
                        const bool depth_write = true, const float clear_depth = 1.0f,
                        const glm::vec2 pixel_offset = glm::vec2(0)) {
    std::array<CameraInfoBlock, 2> cameras{};
    cameras[0].view = cameras[0].inverse_view = glm::mat4(1);
    cameras[0].projection = perspective ? glm::perspectiveRH_ZO(glm::half_pi<float>(), 1.0f, 0.1f, 100.0f)
                                        : glm::orthoRH_ZO(-1.0f, 1.0f, -1.0f, 1.0f, 0.1f, 100.0f);
    cameras[0].resolution = glm::vec2(999);  // The actual viewport must determine pixel diameter.
    cameras[1] = cameras[0];
    cameras[1].projection[0][0] *= 2;
    cameras[1].projection[1][1] *= 2;
    cameras_buffer->Upload(cameras);
    StarClusterGpuResult star{};
    const auto& projection = cameras[push.camera_index].projection;
    const double clip_w = perspective ? distance : 1;
    star.world_position_radius = {pixel_offset.x * 2 * clip_w / (push.viewport_size.x * projection[0][0]),
                                  pixel_offset.y * 2 * clip_w / (push.viewport_size.y * projection[1][1]), -distance,
                                  radius};
    star.color_emission = {2, 0.5f, 1, 1000};
    stars_buffer->Upload(std::array{star});
    const auto width = static_cast<uint32_t>(push.viewport_size.x);
    const auto height = static_cast<uint32_t>(push.viewport_size.y);
    ConfigureStarRenderStates(pipeline, {0, 0, width, height}, depth_write);
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      target.GetColorImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      target.GetDepthImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      std::vector<VkRenderingAttachmentInfo> colors;
      target.AppendColorAttachmentInfos(colors, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      auto depth = target.GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      depth.clearValue.depthStencil.depth = clear_depth;
      VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
      rendering.renderArea = {{0, 0}, {width, height}};
      rendering.layerCount = 1;
      rendering.colorAttachmentCount = 1;
      rendering.pColorAttachments = colors.data();
      rendering.pDepthAttachment = &depth;
      Platform::BeginRendering(command, rendering);
      pipeline.states.ApplyAllStates(command);
      pipeline.Bind(command);
      pipeline.BindDescriptorSet(command, 0, camera_descriptor->GetVkDescriptorSet());
      pipeline.BindDescriptorSet(command, 1, star_descriptor->GetVkDescriptorSet());
      pipeline.PushConstant(command, 0, push);
      Platform::Draw(command, 6, 1);
      Platform::EndRendering(command);
      target.GetColorImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    });
    std::vector<glm::vec4> pixels;
    target.GetRgbaChannelData(pixels);
    EXPECT_EQ(glm::vec4(0), pixels[0]);
    last_pixels = pixels;
    return pixels[(height / 2) * 65 + width / 2];
  };
  const glm::vec4 limited(0.8991f, 0.8991f * 0.25f, 0.8991f * 0.5f, 1);
  const glm::vec4 emissive(2000, 500, 1000, 1);
  for (const float strength : {0.0f, 0.5f, 1.0f, 2.0f, 4.0f}) {
    const auto pixel = draw(true, 0.5 / 33, 1, {0, 0.8991f, {33, 33}, strength});
    EXPECT_NEAR(pixel.x, limited.x * std::pow(0.25f, strength), 1e-6);
    EXPECT_LE(pixel.x, limited.x);
    EXPECT_EQ(emissive, draw(true, 2.0 / 33, 1, {0, 0.8991f, {33, 33}, strength}));
  }
  for (const bool perspective : {false, true}) {
    for (const double diameter : {0.5, 0.99, 1.0, 1.01, 2.0}) {
      const auto pixel = draw(perspective, diameter / 33, 1, {});
      const auto expected = diameter < 1 ? glm::vec4(glm::vec3(limited) * float(diameter * diameter), 1) : emissive;
      for (int channel = 0; channel < 4; ++channel)
        EXPECT_NEAR(pixel[channel], expected[channel], 1e-5) << perspective << ", diameter " << diameter;
    }
  }
  EXPECT_EQ(glm::vec4(glm::vec3(limited) * (0.75f * 0.75f), 1),
            draw(true, 1.5 / 33, 2, {}));              // Distance changes perspective size.
  EXPECT_EQ(emissive, draw(false, 1.5 / 33, 2, {}));   // Not orthographic size.
  EXPECT_EQ(emissive, draw(true, 0.75 / 33, 1, {1}));  // Same star, different camera FOV.
  EXPECT_EQ(emissive, draw(true, 0.75 / 33, 1, {0, 0.8991f, {65, 65}}));
  EXPECT_EQ(glm::vec4(glm::vec3(limited) * 0.75f, 1),
            draw(true, 0.75 / 33, 1, {0, 0.8991f, {65, 33}}));  // Narrowest axis.
  EXPECT_EQ(glm::vec4(0.04995f, 0.0124875f, 0.024975f, 1), draw(true, 0.5 / 33, 1, {0, 0.1998f}));
  EXPECT_EQ(glm::vec4(0, 0, 0, 1), draw(true, 0.5 / 33, 1, {0, 0}));
  EXPECT_EQ(glm::vec4(0.25f, 0.0625f, 0.125f, 1), draw(true, 0.5 / 33, 1, {0, 1}));  // Bloom disabled.
  EXPECT_EQ(glm::vec4(glm::vec3(limited) * 0.25f, 1), draw(true, 0.5 / 33, 1, {}, false));
  EXPECT_EQ(glm::vec4(0), draw(true, 0.5 / 33, 1, {}, true, 0));
  EXPECT_EQ(glm::vec4(0),
            draw(true, 0, 1, {}));  // Zero-radius stars remain degenerate, not black depth-writing pixels.
  for (const double distance : {1.0, 2.0, 10.0}) {
    for (const float offset : {-0.49f, 0.0f, 0.49f}) {
      const auto pixel = draw(true, 0.1 / 33, distance, {}, true, 1, {offset, offset});
      EXPECT_NEAR(pixel.x, limited.x * 0.01 / (distance * distance), 1e-6);
      EXPECT_EQ(std::count_if(last_pixels.begin(), last_pixels.end(),
                              [](const auto& p) {
                                return p.w > 0;
                              }),
                1);
    }
  }
  for (const double diameter : {0.01, 0.99, 1.0, 1.01, 1.4}) {
    draw(true, diameter / 33, 1, {}, true, 1, {0.49f, 0.49f});
    EXPECT_EQ(std::count_if(last_pixels.begin(), last_pixels.end(),
                            [](const auto& p) {
                              return p.w > 0;
                            }),
              1);
  }
}

TEST(UniverseStarCluster, HoverRingUsesDisplayPixelsCurrentGpuIndexAndOpaqueDepth) {
  UniverseGpuFixture gpu;
  if (!Platform::GetSelectedPhysicalDevice()->features.shaderFloat64)
    GTEST_SKIP() << "shaderFloat64 unavailable";
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  Shader::RegisterShaderIncludePath(root / "EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules");
  const auto shaders = root / "EvoEngine_Packages/Universe/Internals/UniverseResources/Shaders/Graphics";
  auto vertex = std::make_shared<Shader>();
  auto fragment = std::make_shared<Shader>();
  ASSERT_TRUE(
      vertex->TryCompile(ShaderType::Vertex, Platform::GetShaderGlobalDefines(), shaders / "StarHover.vert.slang"));
  ASSERT_TRUE(
      fragment->TryCompile(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), shaders / "StarHover.frag.slang"));
  auto camera_layout = std::make_shared<DescriptorSetLayout>();
  camera_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
  camera_layout->Initialize();
  auto star_layout = std::make_shared<DescriptorSetLayout>();
  star_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
  star_layout->Initialize();
  const std::vector<std::shared_ptr<DescriptorSetLayout>> layouts{camera_layout, star_layout};
  const std::vector<VkPushConstantRange> ranges{{VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(StarHoverPushConstant)}};
  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Vertex, vertex->PeekShaderCode(), reflection, diagnostics,
                                   shaders / "StarHover.vert.slang"))
      << diagnostics;
  ASSERT_EQ(1u, reflection.push_constant_ranges.size());
  EXPECT_EQ(32u, reflection.push_constant_ranges[0].size);
  for (const auto binding : {glm::uvec2(0, 2), glm::uvec2(1, 0)}) {
    const auto found = std::find_if(reflection.descriptor_bindings.begin(), reflection.descriptor_bindings.end(),
                                    [&](const auto& item) {
                                      return item.set == binding.x && item.binding == binding.y;
                                    });
    ASSERT_NE(found, reflection.descriptor_bindings.end());
    EXPECT_EQ(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, found->descriptor_type);
  }
  auto cameras_buffer = TestBuffer(sizeof(CameraInfoBlock));
  auto stars_buffer = TestBuffer(2 * sizeof(StarClusterGpuResult));
  auto camera_descriptor = std::make_shared<DescriptorSet>(camera_layout);
  auto star_descriptor = std::make_shared<DescriptorSet>(star_layout);
  camera_descriptor->UpdateBufferDescriptorBinding(2, cameras_buffer);
  star_descriptor->UpdateBufferDescriptorBinding(0, stars_buffer);
  CameraInfoBlock camera{};
  camera.view = camera.inverse_view = glm::mat4(1);
  camera.projection = glm::perspectiveRH_ZO(glm::half_pi<float>(), 1.0f, 0.1f, 100.0f);
  cameras_buffer->Upload(camera);
  GraphicsPipeline pipeline;
  pipeline.view_mask = 0;
  pipeline.vertex_shader = vertex;
  pipeline.fragment_shader = fragment;
  pipeline.geometry_type = GeometryType::Mesh;
  pipeline.vertex_input_enabled = false;
  pipeline.primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  pipeline.depth_attachment_format = Platform::Constants::render_texture_depth;
  pipeline.stencil_attachment_format = VK_FORMAT_UNDEFINED;
  pipeline.color_attachment_formats = {VK_FORMAT_R32G32B32A32_SFLOAT};
  pipeline.descriptor_set_layouts = layouts;
  pipeline.push_constant_ranges = ranges;
  pipeline.Initialize();
  ASSERT_TRUE(pipeline.Initialized());
  RenderTextureCreateInfo info;
  info.extent = {65, 65, 1};
  info.color_format = VK_FORMAT_R32G32B32A32_SFLOAT;
  RenderTexture target(info);
  const auto draw = [&](const double radius, const glm::vec2 display_size = glm::vec2(65),
                        const float clear_depth = 1.0f, const float brightness = 0.4f) {
    std::array<StarClusterGpuResult, 2> stars{};
    stars[0].world_position_radius = {1000, 1000, -1, radius};
    stars[1].world_position_radius = {0, 0, -1, radius};
    stars_buffer->Upload(stars);
    ConfigureStarRenderStates(pipeline, {0, 0, 65, 65}, false);
    auto& blend = pipeline.states.color_blend_attachment_states[0];
    blend.blendEnable = VK_TRUE;
    blend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    blend.colorBlendOp = VK_BLEND_OP_ADD;
    blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    blend.alphaBlendOp = VK_BLEND_OP_ADD;
    EXPECT_FALSE(pipeline.states.depth_write);
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      target.GetColorImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      target.GetDepthImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      std::vector<VkRenderingAttachmentInfo> colors;
      target.AppendColorAttachmentInfos(colors, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      auto depth = target.GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      depth.clearValue.depthStencil.depth = clear_depth;
      VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
      rendering.renderArea = {{0, 0}, {65, 65}};
      rendering.layerCount = 1;
      rendering.colorAttachmentCount = 1;
      rendering.pColorAttachments = colors.data();
      rendering.pDepthAttachment = &depth;
      Platform::BeginRendering(command, rendering);
      pipeline.states.ApplyAllStates(command);
      pipeline.Bind(command);
      pipeline.BindDescriptorSet(command, 0, camera_descriptor->GetVkDescriptorSet());
      pipeline.BindDescriptorSet(command, 1, star_descriptor->GetVkDescriptorSet());
      pipeline.PushConstant(command, 0, StarHoverPushConstant{0, brightness, {65, 65}, display_size, 3, 1});
      Platform::Draw(command, 6, 1);
      Platform::EndRendering(command);
      target.GetColorImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    });
    std::vector<glm::vec4> pixels;
    target.GetRgbaChannelData(pixels);
    return pixels;
  };
  const auto minimum = draw(0.001);
  EXPECT_EQ(glm::vec4(0), minimum[32 * 65 + 32]);
  EXPECT_EQ(glm::vec4(0), minimum[32 * 65 + 36]);  // Empty gap inside the ring.
  EXPECT_NEAR(0.4f, minimum[32 * 65 + 38].y, 1e-4);
  EXPECT_EQ(0, minimum[32 * 65 + 38].x);
  for (const auto& pixel : minimum) {
    EXPECT_LE(pixel.y, 0.40001f);
    EXPECT_EQ(pixel.y, pixel.z);
    EXPECT_EQ(0, pixel.w);  // Preserve destination alpha.
  }
  const auto large = draw(20.0 / 65);
  EXPECT_EQ(glm::vec4(0), large[32 * 65 + 42]);
  EXPECT_NEAR(0.4f, large[32 * 65 + 45].y, 1e-4);
  const auto stretched = draw(0.001, {65, 130});
  EXPECT_GT(stretched[35 * 65 + 32].y, 0.3f);  // Six display pixels = three framebuffer pixels.
  EXPECT_EQ(glm::vec4(0), stretched[38 * 65 + 32]);
  for (const auto& pixel : draw(0.001, {65, 65}, 0))
    EXPECT_EQ(glm::vec4(0), pixel);
  for (const auto& pixel : draw(0.001, {65, 65}, 1, 0))
    EXPECT_EQ(glm::vec4(0), pixel);
}
