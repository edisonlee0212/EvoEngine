#include "PlantArchitecture.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

#ifndef HELIOS_GIT_COMMIT_HASH
#  define HELIOS_GIT_COMMIT_HASH "unknown"
#endif

struct Options {
  std::filesystem::path output;
  unsigned int seed = 1;
  float age_days = 45.f;
  std::optional<int> max_nodes;
  std::optional<float> internode_length_m;
  std::optional<float> leaf_size_min_m;
  std::optional<float> leaf_size_max_m;
  std::optional<float> leaf_aspect_ratio_min;
  std::optional<float> leaf_aspect_ratio_max;
  std::optional<float> petiole_length_min_m;
  std::optional<float> petiole_length_max_m;
  std::optional<unsigned int> leaf_subdivisions;
};

void printUsage() {
  std::cout << "Usage: helios_soybean_exporter --output FILE.obj [options]\n"
            << "  --seed N                         Random seed (default: 1)\n"
            << "  --age-days N                     Plant age in days (default: 45)\n"
            << "  --max-nodes N                    Trifoliate shoot node limit\n"
            << "  --internode-length-m N           Mature internode length\n"
            << "  --leaf-size-min-m N              Minimum mature leaflet scale\n"
            << "  --leaf-size-max-m N              Maximum mature leaflet scale\n"
            << "  --leaf-aspect-ratio-min N        Minimum leaflet width/length ratio\n"
            << "  --leaf-aspect-ratio-max N        Maximum leaflet width/length ratio\n"
            << "  --petiole-length-min-m N         Minimum mature petiole length\n"
            << "  --petiole-length-max-m N         Maximum mature petiole length\n"
            << "  --leaf-subdivisions N            Leaf prototype subdivisions\n";
}

std::string requireValue(int &index, int argc, char **argv) {
  if (++index >= argc) {
    throw std::runtime_error(std::string("Missing value for ") + argv[index - 1]);
  }
  return argv[index];
}

int parseInt(const std::string &text, const std::string &name) {
  std::size_t consumed = 0;
  const int value = std::stoi(text, &consumed);
  if (consumed != text.size()) {
    throw std::runtime_error("Invalid integer for " + name + ": " + text);
  }
  return value;
}

unsigned int parseUnsigned(const std::string &text, const std::string &name) {
  std::size_t consumed = 0;
  const unsigned long value = std::stoul(text, &consumed);
  if (consumed != text.size() || value > std::numeric_limits<unsigned int>::max()) {
    throw std::runtime_error("Invalid unsigned integer for " + name + ": " + text);
  }
  return static_cast<unsigned int>(value);
}

float parseFloat(const std::string &text, const std::string &name) {
  std::size_t consumed = 0;
  const float value = std::stof(text, &consumed);
  if (consumed != text.size()) {
    throw std::runtime_error("Invalid number for " + name + ": " + text);
  }
  return value;
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string name = argv[i];
    if (name == "--help" || name == "-h") {
      printUsage();
      std::exit(0);
    } else if (name == "--version") {
      std::cout << HELIOS_GIT_COMMIT_HASH << '\n';
      std::exit(0);
    } else if (name == "--output") {
      options.output = requireValue(i, argc, argv);
    } else if (name == "--seed") {
      options.seed = parseUnsigned(requireValue(i, argc, argv), name);
    } else if (name == "--age-days") {
      options.age_days = parseFloat(requireValue(i, argc, argv), name);
    } else if (name == "--max-nodes") {
      options.max_nodes = parseInt(requireValue(i, argc, argv), name);
    } else if (name == "--internode-length-m") {
      options.internode_length_m = parseFloat(requireValue(i, argc, argv), name);
    } else if (name == "--leaf-size-min-m") {
      options.leaf_size_min_m = parseFloat(requireValue(i, argc, argv), name);
    } else if (name == "--leaf-size-max-m") {
      options.leaf_size_max_m = parseFloat(requireValue(i, argc, argv), name);
    } else if (name == "--leaf-aspect-ratio-min") {
      options.leaf_aspect_ratio_min = parseFloat(requireValue(i, argc, argv), name);
    } else if (name == "--leaf-aspect-ratio-max") {
      options.leaf_aspect_ratio_max = parseFloat(requireValue(i, argc, argv), name);
    } else if (name == "--petiole-length-min-m") {
      options.petiole_length_min_m = parseFloat(requireValue(i, argc, argv), name);
    } else if (name == "--petiole-length-max-m") {
      options.petiole_length_max_m = parseFloat(requireValue(i, argc, argv), name);
    } else if (name == "--leaf-subdivisions") {
      options.leaf_subdivisions = parseUnsigned(requireValue(i, argc, argv), name);
    } else {
      throw std::runtime_error("Unknown option: " + name);
    }
  }

  if (options.output.empty()) {
    throw std::runtime_error("--output is required");
  }
  if (options.age_days < 0.f) {
    throw std::runtime_error("--age-days must be non-negative");
  }
  if (options.max_nodes && *options.max_nodes <= 0) {
    throw std::runtime_error("--max-nodes must be positive");
  }
  if (options.internode_length_m && *options.internode_length_m <= 0.f) {
    throw std::runtime_error("--internode-length-m must be positive");
  }
  if (options.leaf_size_min_m.has_value() != options.leaf_size_max_m.has_value()) {
    throw std::runtime_error("Both leaf size bounds must be specified together");
  }
  if (options.leaf_size_min_m &&
      (*options.leaf_size_min_m <= 0.f || *options.leaf_size_min_m > *options.leaf_size_max_m)) {
    throw std::runtime_error("Leaf size range must be positive and ordered");
  }
  if (options.leaf_aspect_ratio_min.has_value() != options.leaf_aspect_ratio_max.has_value()) {
    throw std::runtime_error("Both leaf aspect ratio bounds must be specified together");
  }
  if (options.leaf_aspect_ratio_min &&
      (*options.leaf_aspect_ratio_min <= 0.f || *options.leaf_aspect_ratio_min > *options.leaf_aspect_ratio_max)) {
    throw std::runtime_error("Leaf aspect ratio range must be positive and ordered");
  }
  if (options.petiole_length_min_m.has_value() != options.petiole_length_max_m.has_value()) {
    throw std::runtime_error("Both petiole length bounds must be specified together");
  }
  if (options.petiole_length_min_m &&
      (*options.petiole_length_min_m <= 0.f || *options.petiole_length_min_m > *options.petiole_length_max_m)) {
    throw std::runtime_error("Petiole length range must be positive and ordered");
  }
  if (options.leaf_subdivisions && *options.leaf_subdivisions == 0) {
    throw std::runtime_error("--leaf-subdivisions must be positive");
  }
  return options;
}

bool hasParameterOverrides(const Options &options) {
  return options.max_nodes || options.internode_length_m || options.leaf_size_min_m || options.leaf_aspect_ratio_min ||
         options.petiole_length_min_m || options.leaf_subdivisions;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parseOptions(argc, argv);
    if (!options.output.parent_path().empty()) {
      std::filesystem::create_directories(options.output.parent_path());
    }

    helios::Context context;
    context.seedRandomGenerator(options.seed);
    PlantArchitecture plants(&context);
    plants.loadPlantModelFromLibrary("soybean");

    if (hasParameterOverrides(options)) {
      ShootParameters parameters = plants.getCurrentShootParameters("trifoliate");
      if (options.max_nodes) {
        parameters.max_nodes = *options.max_nodes;
      }
      if (options.internode_length_m) {
        parameters.internode_length_max = *options.internode_length_m;
      }
      if (options.leaf_size_min_m) {
        parameters.phytomer_parameters.leaf.prototype_scale.uniformDistribution(*options.leaf_size_min_m,
                                                                                *options.leaf_size_max_m);
      }
      if (options.leaf_aspect_ratio_min) {
        parameters.phytomer_parameters.leaf.prototype.leaf_aspect_ratio.uniformDistribution(
            *options.leaf_aspect_ratio_min, *options.leaf_aspect_ratio_max);
      }
      if (options.petiole_length_min_m) {
        parameters.phytomer_parameters.petiole.length.uniformDistribution(*options.petiole_length_min_m,
                                                                          *options.petiole_length_max_m);
      }
      if (options.leaf_subdivisions) {
        parameters.phytomer_parameters.leaf.prototype.subdivisions = *options.leaf_subdivisions;
      }
      plants.updateCurrentShootParameters("trifoliate", parameters);

      // Keep the two small unifoliate leaves consistent with the selected soybean leaf-shape variant.
      if (options.leaf_aspect_ratio_min) {
        ShootParameters unifoliate_parameters = plants.getCurrentShootParameters("unifoliate");
        unifoliate_parameters.phytomer_parameters.leaf.prototype.leaf_aspect_ratio.uniformDistribution(
            *options.leaf_aspect_ratio_min, *options.leaf_aspect_ratio_max);
        plants.updateCurrentShootParameters("unifoliate", unifoliate_parameters);
      }
    }

    const unsigned int plant_id =
        plants.buildPlantInstanceFromLibrary(helios::make_vec3(0.f, 0.f, 0.f), options.age_days);
    const std::vector<unsigned int> primitive_ids = plants.getAllPlantUUIDs(plant_id);
    context.writeOBJ(options.output.string(), primitive_ids, true, false);

    // Record realized growth, separately from the requested prototype parameters.
    auto metadata_path = options.output;
    metadata_path.replace_extension(".geometry.json");
    std::ofstream metadata(metadata_path);
    metadata << std::setprecision(9) << "{\n  \"seed\": " << options.seed << ",\n  \"age_days\": " << options.age_days
             << ",\n  \"primitive_count\": " << primitive_ids.size()
             << ",\n  \"leaf_count\": " << plants.getPlantLeafCount(plant_id)
             << ",\n  \"height_m\": " << plants.getPlantHeight(plant_id)
             << ",\n  \"helios_leaf_area_m2\": " << plants.sumPlantLeafArea(plant_id) << "\n}\n";

    if (!std::filesystem::exists(options.output)) {
      throw std::runtime_error("Helios did not create the requested OBJ file");
    }
    std::cout << "Exported " << primitive_ids.size() << " primitives to " << options.output << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "helios_soybean_exporter: " << error.what() << '\n';
    return 1;
  }
}
