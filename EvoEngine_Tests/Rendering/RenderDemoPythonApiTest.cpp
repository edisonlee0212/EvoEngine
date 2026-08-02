#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace {
constexpr double kMinimumPsnr = 30.0;
constexpr double kMinimumSsim = 0.95;
constexpr int kRayTargetSamples = 2048;
constexpr int kRaySamplesPerFrame = 8;
constexpr int kRayAccumulationFrames = kRayTargetSamples / kRaySamplesPerFrame;
static_assert(kRayAccumulationFrames * kRaySamplesPerFrame == kRayTargetSamples);

struct RenderCaptureCase {
  const char* render_mode;
  const char* output_file_name;
  const char* baseline_file_name;
  const char* artifact_file_name;
  int width;
  int height;
  int warmup_frames;
  int accumulation_frames;
  int samples_per_frame;
  int bounces;
};

constexpr RenderCaptureCase kRasterizationCapture{
    "Rasterization",
    "rendering_demo_rasterization.png",
    "RenderingDemo.CapturesSceneThroughPythonApi.2560x1440.png",
    "RenderingDemo.CapturesSceneThroughPythonApi.png",
    2560,
    1440,
    1800,
    0,
    4,
    4,
};
constexpr RenderCaptureCase kRayTracingCapture{
    "RayTracing",
    "rendering_demo_ray_tracing.png",
    "RenderingDemo.RayTracingGoldenImage.1280x720.2048spp.png",
    "RenderingDemo.RayTracingGoldenImage.png",
    1280,
    720,
    0,
    kRayAccumulationFrames,
    kRaySamplesPerFrame,
    3,
};
constexpr RenderCaptureCase kRayQueryCapture{
    "RayQuery",
    "rendering_demo_ray_query.png",
    "RenderingDemo.RayQueryGoldenImage.1280x720.2048spp.png",
    "RenderingDemo.RayQueryGoldenImage.png",
    1280,
    720,
    0,
    kRayAccumulationFrames,
    kRaySamplesPerFrame,
    3,
};

struct Image {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> rgba;
};

std::string Quote(const std::filesystem::path& path) {
  std::string value = path.string();
  std::string quoted = "\"";
  for (const char c : value) {
    if (c == '"') {
      quoted += "\\\"";
    } else {
      quoted += c;
    }
  }
  quoted += "\"";
  return quoted;
}

std::filesystem::path CreateTestDirectory(const std::filesystem::path& executable_dir) {
  const auto test_root = executable_dir / "RenderingDemoPythonApiTest";
  std::filesystem::create_directories(test_root);

  const auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
  for (int attempt = 0; attempt < 16; attempt++) {
    const auto test_dir = test_root / (std::to_string(timestamp) + "-" + std::to_string(attempt));
    if (std::filesystem::create_directories(test_dir)) {
      return test_dir;
    }
  }

  throw std::runtime_error("Failed to create render test output directory.");
}

Image LoadPng(const std::filesystem::path& image_path) {
  int width = 0;
  int height = 0;
  int channels = 0;
  stbi_uc* pixels = stbi_load(image_path.string().c_str(), &width, &height, &channels, 4);
  if (pixels == nullptr) {
    const char* reason = stbi_failure_reason();
    throw std::runtime_error("Failed to decode PNG '" + image_path.string() +
                             "': " + (reason == nullptr ? "unknown stb_image error" : reason));
  }

  Image image;
  image.width = width;
  image.height = height;
  const auto size = static_cast<size_t>(width) * static_cast<size_t>(height) * 4u;
  image.rgba.assign(pixels, pixels + size);
  stbi_image_free(pixels);
  return image;
}

void AssertImageIsRenderablePng(const std::filesystem::path& output_path, const Image& image, const int expected_width,
                                const int expected_height) {
  ASSERT_TRUE(std::filesystem::exists(output_path));
  ASSERT_GT(std::filesystem::file_size(output_path), 0u);
  EXPECT_EQ(image.width, expected_width);
  EXPECT_EQ(image.height, expected_height);

  const auto pixel_count = static_cast<size_t>(image.width) * static_cast<size_t>(image.height);
  ASSERT_GT(pixel_count, 0u);
  const auto first_r = image.rgba[0];
  const auto first_g = image.rgba[1];
  const auto first_b = image.rgba[2];
  bool differs_from_first = false;
  bool has_non_black_rgb = false;
  for (size_t i = 0; i < pixel_count; i++) {
    const auto r = image.rgba[i * 4];
    const auto g = image.rgba[i * 4 + 1];
    const auto b = image.rgba[i * 4 + 2];
    differs_from_first = differs_from_first || r != first_r || g != first_g || b != first_b;
    has_non_black_rgb = has_non_black_rgb || r != 0 || g != 0 || b != 0;
  }

  EXPECT_TRUE(has_non_black_rgb);
  EXPECT_TRUE(differs_from_first);
}

std::array<double, 11> GaussianKernel() {
  constexpr int radius = 5;
  constexpr double sigma = 1.5;
  std::array<double, 11> kernel{};
  double sum = 0.0;
  for (int offset = -radius; offset <= radius; offset++) {
    const auto value = std::exp(-(static_cast<double>(offset * offset)) / (2.0 * sigma * sigma));
    kernel[static_cast<size_t>(offset + radius)] = value;
    sum += value;
  }
  for (auto& value : kernel) {
    value /= sum;
  }
  return kernel;
}

std::vector<float> GaussianBlur(const std::vector<float>& input, const int width, const int height) {
  const auto kernel = GaussianKernel();
  const auto pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);
  std::vector<float> temp(pixel_count);
  std::vector<float> output(pixel_count);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double value = 0.0;
      for (int offset = -5; offset <= 5; offset++) {
        const int sample_x = std::clamp(x + offset, 0, width - 1);
        value += kernel[static_cast<size_t>(offset + 5)] *
                 input[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(sample_x)];
      }
      temp[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] = static_cast<float>(value);
    }
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double value = 0.0;
      for (int offset = -5; offset <= 5; offset++) {
        const int sample_y = std::clamp(y + offset, 0, height - 1);
        value += kernel[static_cast<size_t>(offset + 5)] *
                 temp[static_cast<size_t>(sample_y) * static_cast<size_t>(width) + static_cast<size_t>(x)];
      }
      output[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] = static_cast<float>(value);
    }
  }

  return output;
}

double CalculatePsnr(const Image& actual, const Image& expected) {
  const auto pixel_count = static_cast<size_t>(actual.width) * static_cast<size_t>(actual.height);
  double squared_error = 0.0;
  for (size_t i = 0; i < pixel_count; i++) {
    for (size_t channel = 0; channel < 3; channel++) {
      const double difference =
          static_cast<double>(actual.rgba[i * 4 + channel]) - static_cast<double>(expected.rgba[i * 4 + channel]);
      squared_error += difference * difference;
    }
  }

  const double mean_squared_error = squared_error / (static_cast<double>(pixel_count) * 3.0);
  if (mean_squared_error == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return 10.0 * std::log10((255.0 * 255.0) / mean_squared_error);
}

double CalculateSsim(const Image& actual, const Image& expected) {
  const auto pixel_count = static_cast<size_t>(actual.width) * static_cast<size_t>(actual.height);
  std::vector<float> actual_luminance(pixel_count);
  std::vector<float> expected_luminance(pixel_count);
  std::vector<float> actual_squared(pixel_count);
  std::vector<float> expected_squared(pixel_count);
  std::vector<float> cross(pixel_count);

  for (size_t i = 0; i < pixel_count; i++) {
    const float actual_y = 0.2126f * static_cast<float>(actual.rgba[i * 4]) +
                           0.7152f * static_cast<float>(actual.rgba[i * 4 + 1]) +
                           0.0722f * static_cast<float>(actual.rgba[i * 4 + 2]);
    const float expected_y = 0.2126f * static_cast<float>(expected.rgba[i * 4]) +
                             0.7152f * static_cast<float>(expected.rgba[i * 4 + 1]) +
                             0.0722f * static_cast<float>(expected.rgba[i * 4 + 2]);
    actual_luminance[i] = actual_y;
    expected_luminance[i] = expected_y;
    actual_squared[i] = actual_y * actual_y;
    expected_squared[i] = expected_y * expected_y;
    cross[i] = actual_y * expected_y;
  }

  const auto actual_mean = GaussianBlur(actual_luminance, actual.width, actual.height);
  const auto expected_mean = GaussianBlur(expected_luminance, actual.width, actual.height);
  const auto actual_squared_mean = GaussianBlur(actual_squared, actual.width, actual.height);
  const auto expected_squared_mean = GaussianBlur(expected_squared, actual.width, actual.height);
  const auto cross_mean = GaussianBlur(cross, actual.width, actual.height);

  constexpr double c1 = 0.01 * 255.0 * 0.01 * 255.0;
  constexpr double c2 = 0.03 * 255.0 * 0.03 * 255.0;
  double ssim_sum = 0.0;
  for (size_t i = 0; i < pixel_count; i++) {
    const double mu_actual = actual_mean[i];
    const double mu_expected = expected_mean[i];
    const double sigma_actual_squared =
        std::max(0.0, static_cast<double>(actual_squared_mean[i]) - mu_actual * mu_actual);
    const double sigma_expected_squared =
        std::max(0.0, static_cast<double>(expected_squared_mean[i]) - mu_expected * mu_expected);
    const double sigma_cross = static_cast<double>(cross_mean[i]) - mu_actual * mu_expected;

    const double numerator = (2.0 * mu_actual * mu_expected + c1) * (2.0 * sigma_cross + c2);
    const double denominator =
        (mu_actual * mu_actual + mu_expected * mu_expected + c1) * (sigma_actual_squared + sigma_expected_squared + c2);
    ssim_sum += denominator == 0.0 ? 1.0 : numerator / denominator;
  }

  return ssim_sum / static_cast<double>(pixel_count);
}

void AcceptBaselineIfRequested(const std::filesystem::path& output_path, const std::filesystem::path& baseline_path) {
  const char* accept_baseline_env = std::getenv("EVOENGINE_ACCEPT_RENDER_BASELINE");
  if (accept_baseline_env == nullptr || accept_baseline_env[0] == '\0' || std::string(accept_baseline_env) == "0") {
    return;
  }

  std::filesystem::create_directories(baseline_path.parent_path());
  std::filesystem::copy_file(output_path, baseline_path, std::filesystem::copy_options::overwrite_existing);
  std::cout << "Accepted render baseline: " << baseline_path << std::endl;
}

void AssertGoldenImage(const Image& actual, const std::filesystem::path& baseline_path) {
  ASSERT_TRUE(std::filesystem::exists(baseline_path))
      << "Missing render baseline. Regenerate it with: python .\\test.py --render-only --accept-render-baseline";

  const Image expected = LoadPng(baseline_path);
  ASSERT_EQ(expected.width, actual.width);
  ASSERT_EQ(expected.height, actual.height);

  const double psnr = CalculatePsnr(actual, expected);
  const double ssim = CalculateSsim(actual, expected);
  std::cout << "Render comparison PSNR: " << psnr << " dB" << std::endl;
  std::cout << "Render comparison SSIM: " << ssim << std::endl;
  EXPECT_GE(psnr, kMinimumPsnr);
  EXPECT_GE(ssim, kMinimumSsim);
}

void CopyVisualArtifact(const std::filesystem::path& output_path, const char* artifact_file_name) {
  const char* artifact_dir_env = std::getenv("EVOENGINE_TEST_ARTIFACT_DIR");
  if (artifact_dir_env == nullptr || artifact_dir_env[0] == '\0') {
    return;
  }

  const auto artifact_dir = std::filesystem::path(artifact_dir_env);
  std::filesystem::create_directories(artifact_dir);
  std::filesystem::copy_file(output_path, artifact_dir / artifact_file_name,
                             std::filesystem::copy_options::overwrite_existing);
}

void RunRenderingDemoCapture(const RenderCaptureCase& capture) {
  const std::filesystem::path executable_dir = std::filesystem::path(EVOENGINE_RENDER_TEST_DIR);
  const std::filesystem::path test_dir = CreateTestDirectory(executable_dir);
  const std::filesystem::path output_path = test_dir / capture.output_file_name;
  const std::filesystem::path source_resources_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Resources";
  const std::filesystem::path test_resources_root = test_dir / "Resources";
  const std::filesystem::path script_path = std::filesystem::path(EVOENGINE_TEST_SCRIPT_DIR) / "render_demo_capture.py";
  const std::filesystem::path baseline_path =
      std::filesystem::path(EVOENGINE_TEST_SCRIPT_DIR) / "Baselines" / capture.baseline_file_name;

  ASSERT_TRUE(std::filesystem::exists(script_path));
  ASSERT_TRUE(std::filesystem::exists(source_resources_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets"));

  std::string command = Quote(EVOENGINE_TEST_PYTHON_EXECUTABLE);
  command += " " + Quote(script_path);
  command += " --module-dir " + Quote(EVOENGINE_PYEVOENGINE_DIR);
  command += " --source-resources-root " + Quote(source_resources_root);
  command += " --test-resources-root " + Quote(test_resources_root);
  command += " --output " + Quote(output_path);
  command += " --width " + std::to_string(capture.width) + " --height " + std::to_string(capture.height);
  command += " --render-mode " + std::string(capture.render_mode);
  command += " --samples-per-frame " + std::to_string(capture.samples_per_frame);
  command += " --bounces " + std::to_string(capture.bounces);
  if (capture.accumulation_frames > 0) {
    command += " --accumulation-frames " + std::to_string(capture.accumulation_frames);
  } else {
    command += " --warmup-frames " + std::to_string(capture.warmup_frames);
  }

#ifdef _WIN32
  const std::string system_command = "\"" + command + "\"";
#else
  const std::string system_command = command;
#endif
  const int exit_code = std::system(system_command.c_str());
  ASSERT_EQ(exit_code, 0) << system_command;
  const Image output_image = LoadPng(output_path);
  ASSERT_NO_FATAL_FAILURE(AssertImageIsRenderablePng(output_path, output_image, capture.width, capture.height));
  CopyVisualArtifact(output_path, capture.artifact_file_name);
  AcceptBaselineIfRequested(output_path, baseline_path);
  ASSERT_NO_FATAL_FAILURE(AssertGoldenImage(output_image, baseline_path));
}
}  // namespace

TEST(RenderingDemo, CapturesSceneThroughPythonApi) {
  RunRenderingDemoCapture(kRasterizationCapture);
}

TEST(RenderingDemo, RayTracingGoldenImage) {
  RunRenderingDemoCapture(kRayTracingCapture);
}

TEST(RenderingDemo, RayQueryGoldenImage) {
  RunRenderingDemoCapture(kRayQueryCapture);
}
