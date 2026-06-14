#pragma once

#include "BasicPointCloudScanner.hpp"
#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "RayTracerCamera.hpp"
#include "TriangleIlluminationEstimator.hpp"

namespace evo_engine {
void SerializeBtfMeshRenderer(YAML::Emitter& out, const BtfMeshRenderer& target);
void DeserializeBtfMeshRenderer(const YAML::Node& in, BtfMeshRenderer& target);
void SerializeBasicPointCloudScanner(YAML::Emitter& out, const BasicPointCloudScanner& target);
void DeserializeBasicPointCloudScanner(const YAML::Node& in, BasicPointCloudScanner& target);
void SerializeTriangleIlluminationEstimator(YAML::Emitter& out, const TriangleIlluminationEstimator& target);
void DeserializeTriangleIlluminationEstimator(const YAML::Node& in, TriangleIlluminationEstimator& target);
void SerializeRayTracerCamera(YAML::Emitter& out, const RayTracerCamera& target);
void DeserializeRayTracerCamera(const YAML::Node& in, RayTracerCamera& target);
void SerializeBtfMaterial(YAML::Emitter& out, const BtfMaterial& target);
void DeserializeBtfMaterial(const YAML::Node& in, BtfMaterial& target);
}  // namespace evo_engine
