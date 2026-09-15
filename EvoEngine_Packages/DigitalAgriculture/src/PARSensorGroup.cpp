//
// Created by lllll on 2/23/2022.
//
#include <Jobs.hpp>
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "Platform.hpp"

using namespace digital_agriculture_package;

void digital_agriculture_package::SerializePARSensorGroup(YAML::Emitter& out, const PARSensorGroup& target) {
  if (!target.samplers.empty()) {
    out << YAML::Key << "samplers" << YAML::Value
        << YAML::Binary((const unsigned char*)target.samplers.data(),
                        target.samplers.size() * sizeof(IlluminationSampler<glm::vec3>));
  }
}
void digital_agriculture_package::DeserializePARSensorGroup(const YAML::Node& in, PARSensorGroup& target) {
  if (in["samplers"]) {
    const auto binary_list = in["samplers"].as<YAML::Binary>();
    target.samplers.resize(binary_list.size() / sizeof(IlluminationSampler<glm::vec3>));
    std::memcpy(target.samplers.data(), binary_list.data(), binary_list.size());
  }
}
