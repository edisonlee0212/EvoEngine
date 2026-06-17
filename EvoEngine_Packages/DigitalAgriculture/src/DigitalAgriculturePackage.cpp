#include "PackageManager.hpp"

#ifdef CUDA_MODULE_SERVICE
#  include "CBTFImporter.hpp"
#  include "CBTFGroup.hpp"
#  include "LeafIlluminationEstimator.hpp"
#  include "PARSensorGroup.hpp"
#  include "SorghumFieldGrid.hpp"
#endif
#include "CropDescriptor.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "InspectorRegistry.hpp"
#include "Serialization.hpp"
#include "SkyIlluminance.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumField.hpp"
#include "SorghumFieldGrid.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumLayer.hpp"
#include "SorghumState.hpp"
#include "SorghumTraitDescriptor.hpp"

using namespace digital_agriculture_package;
using namespace evo_engine;

namespace {
PackageDescriptor descriptor{EVOENGINE_PACKAGE_API_VERSION, "DigitalAgriculture", "0.1.0",
                             "Digital agriculture runtime package."};

template <typename T>
void RegisterAssetPreviewHandler(const std::string& owner_name, const std::string& type_name) {
  Serialization::RegisterAssetPreviewHandler<T>(
      [](const std::shared_ptr<T>& asset, const OffscreenPreviewSettings&) {
        return asset ? asset->GenerateThumbnailTexture() : nullptr;
      },
      owner_name, type_name);
}

void RegisterDigitalAgricultureAssetPreviewHandlers(const std::string& owner_name) {
  RegisterAssetPreviewHandler<SorghumDescriptor>(owner_name, "SorghumDescriptor");
  RegisterAssetPreviewHandler<SorghumGrowthStages>(owner_name, "SorghumGrowthStages");
  RegisterAssetPreviewHandler<SorghumState>(owner_name, "SorghumState");
  RegisterAssetPreviewHandler<SorghumGenerator>(owner_name, "SorghumGenerator");
  RegisterAssetPreviewHandler<SorghumField>(owner_name, "SorghumField");
}

template <typename T>
void RegisterDrawGuiInspector(const std::string& owner_name, const std::string& type_name) {
  InspectorRegistry::GetInstance().RegisterInspector<T>(
      [](InspectorContext& context, T& target) {
        return target.DrawGui(context.editor_layer);
      },
      owner_name, type_name);
}

void SerializeCropDescriptor(YAML::Emitter& out, const CropDescriptor& target) {
  out << YAML::Key << "base_temperature" << YAML::Value << target.base_temperature;
  out << YAML::Key << "plastochron_gdd" << YAML::Value << target.plastochron_gdd;
  out << YAML::Key << "final_leaf_number" << YAML::Value << target.final_leaf_number;
  out << YAML::Key << "stem_elongation_gdd" << YAML::Value << target.stem_elongation_gdd;
  out << YAML::Key << "flowering_gdd" << YAML::Value << target.flowering_gdd;
  out << YAML::Key << "grain_filling_gdd" << YAML::Value << target.grain_filling_gdd;
  out << YAML::Key << "maturity_gdd" << YAML::Value << target.maturity_gdd;
  out << YAML::Key << "leaf_growth_duration_gdd" << YAML::Value << target.leaf_growth_duration_gdd;
  out << YAML::Key << "senescence_onset_gdd" << YAML::Value << target.senescence_onset_gdd;

  target.max_leaf_length.Save("max_leaf_length", out);
  target.max_leaf_width.Save("max_leaf_width", out);
  target.leaf_sheath_length.Save("leaf_sheath_length", out);
  target.leaf_roll_angle.Save("leaf_roll_angle", out);
  target.leaf_branching_angle.Save("leaf_branching_angle", out);
  target.leaf_curling.Save("leaf_curling", out);
  target.leaf_bending.Save("leaf_bending", out);
  target.leaf_bending_acceleration.Save("leaf_bending_acceleration", out);
  target.leaf_bending_smoothness.Save("leaf_bending_smoothness", out);
  target.leaf_waviness.Save("leaf_waviness", out);
  target.leaf_waviness_frequency.Save("leaf_waviness_frequency", out);
  target.max_internode_length.Save("max_internode_length", out);
  target.max_internode_diameter.Save("max_internode_diameter", out);
  target.stem_tilt_angle.Save("stem_tilt_angle", out);

  target.width_along_stem.Save("width_along_stem", out);
  target.width_along_leaf.Save("width_along_leaf", out);
  target.curling_along_leaf.Save("curling_along_leaf", out);
  target.waviness_along_leaf.Save("waviness_along_leaf", out);

  target.panicle_size.Save("panicle_size", out);
  target.panicle_seed_amount.Save("panicle_seed_amount", out);
  target.panicle_seed_radius.Save("panicle_seed_radius", out);

  out << YAML::Key << "specific_leaf_area" << YAML::Value << target.specific_leaf_area;
  out << YAML::Key << "max_stem_reserve_fraction" << YAML::Value << target.max_stem_reserve_fraction;
}

void DeserializeCropDescriptor(const YAML::Node& in, CropDescriptor& target) {
  if (in["base_temperature"])
    target.base_temperature = in["base_temperature"].as<float>();
  if (in["plastochron_gdd"])
    target.plastochron_gdd = in["plastochron_gdd"].as<float>();
  if (in["final_leaf_number"])
    target.final_leaf_number = in["final_leaf_number"].as<int>();
  if (in["stem_elongation_gdd"])
    target.stem_elongation_gdd = in["stem_elongation_gdd"].as<float>();
  if (in["flowering_gdd"])
    target.flowering_gdd = in["flowering_gdd"].as<float>();
  if (in["grain_filling_gdd"])
    target.grain_filling_gdd = in["grain_filling_gdd"].as<float>();
  if (in["maturity_gdd"])
    target.maturity_gdd = in["maturity_gdd"].as<float>();
  if (in["leaf_growth_duration_gdd"])
    target.leaf_growth_duration_gdd = in["leaf_growth_duration_gdd"].as<float>();
  if (in["senescence_onset_gdd"])
    target.senescence_onset_gdd = in["senescence_onset_gdd"].as<float>();

  target.max_leaf_length.Load("max_leaf_length", in);
  target.max_leaf_width.Load("max_leaf_width", in);
  target.leaf_sheath_length.Load("leaf_sheath_length", in);
  target.leaf_roll_angle.Load("leaf_roll_angle", in);
  target.leaf_branching_angle.Load("leaf_branching_angle", in);
  target.leaf_curling.Load("leaf_curling", in);
  target.leaf_bending.Load("leaf_bending", in);
  target.leaf_bending_acceleration.Load("leaf_bending_acceleration", in);
  target.leaf_bending_smoothness.Load("leaf_bending_smoothness", in);
  target.leaf_waviness.Load("leaf_waviness", in);
  target.leaf_waviness_frequency.Load("leaf_waviness_frequency", in);
  target.max_internode_length.Load("max_internode_length", in);
  target.max_internode_diameter.Load("max_internode_diameter", in);
  target.stem_tilt_angle.Load("stem_tilt_angle", in);

  target.width_along_stem.Load("width_along_stem", in);
  target.width_along_leaf.Load("width_along_leaf", in);
  target.curling_along_leaf.Load("curling_along_leaf", in);
  target.waviness_along_leaf.Load("waviness_along_leaf", in);

  target.panicle_size.Load("panicle_size", in);
  target.panicle_seed_amount.Load("panicle_seed_amount", in);
  target.panicle_seed_radius.Load("panicle_seed_radius", in);

  if (in["specific_leaf_area"])
    target.specific_leaf_area = in["specific_leaf_area"].as<float>();
  if (in["max_stem_reserve_fraction"])
    target.max_stem_reserve_fraction = in["max_stem_reserve_fraction"].as<float>();
}

void SerializeSorghumFieldGrid(YAML::Emitter& out, const SorghumFieldGrid& target) {
  target.sorghum_field_asset.Save("sorghum_field_asset", out);
  out << YAML::Key << "rows" << YAML::Value << target.rows;
  out << YAML::Key << "columns" << YAML::Value << target.columns;
  out << YAML::Key << "row_spacing" << YAML::Value << target.row_spacing;
  out << YAML::Key << "column_spacing" << YAML::Value << target.column_spacing;
  out << YAML::Key << "row_spacing_std" << YAML::Value << target.row_spacing_std;
  out << YAML::Key << "column_spacing_std" << YAML::Value << target.column_spacing_std;
  out << YAML::Key << "sorghum_size" << YAML::Value << target.sorghum_size;
  out << YAML::Key << "size_limit" << YAML::Value << target.size_limit;
  out << YAML::Key << "base_seed" << YAML::Value << target.base_seed;
}

void DeserializeSorghumFieldGrid(const YAML::Node& in, SorghumFieldGrid& target) {
  target.sorghum_field_asset.Load("sorghum_field_asset", in);
  if (in["rows"])
    target.rows = in["rows"].as<int>();
  if (in["columns"])
    target.columns = in["columns"].as<int>();
  if (in["row_spacing"])
    target.row_spacing = in["row_spacing"].as<float>();
  if (in["column_spacing"])
    target.column_spacing = in["column_spacing"].as<float>();
  if (in["row_spacing_std"])
    target.row_spacing_std = in["row_spacing_std"].as<float>();
  if (in["column_spacing_std"])
    target.column_spacing_std = in["column_spacing_std"].as<float>();
  if (in["sorghum_size"])
    target.sorghum_size = in["sorghum_size"].as<float>();
  if (in["size_limit"])
    target.size_limit = in["size_limit"].as<int>();
  if (in["base_seed"])
    target.base_seed = in["base_seed"].as<uint32_t>();
}

void SerializeSorghumTraitDescriptor(YAML::Emitter& out, const SorghumTraitDescriptor& target) {
  out << YAML::Key << "stem_length" << YAML::Value << target.stem_length;
  out << YAML::Key << "leaf_count" << YAML::Value << target.leaf_traits.size();
  if (!target.leaf_traits.empty()) {
    out << YAML::Key << "leaves" << YAML::Value << YAML::BeginSeq;
    for (const auto& leaf : target.leaf_traits) {
      out << YAML::BeginMap;
      leaf.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void DeserializeSorghumTraitDescriptor(const YAML::Node& in, SorghumTraitDescriptor& target) {
  if (in["stem_length"])
    target.stem_length = in["stem_length"].as<float>();
  target.leaf_traits.clear();
  if (in["leaves"]) {
    for (const auto& leaf_node : in["leaves"]) {
      SorghumLeafTrait leaf_trait{};
      leaf_trait.Deserialize(leaf_node);
      target.leaf_traits.push_back(leaf_trait);
    }
  }
}

#ifdef CUDA_MODULE_SERVICE
void SerializeLeafIlluminationEstimator(YAML::Emitter& out, const LeafIlluminationEstimator& target) {
  out << YAML::Key << "leaf_count" << YAML::Value << target.PeekLeafIlluminationInfos().size();
}

void DeserializeLeafIlluminationEstimator(const YAML::Node& in, LeafIlluminationEstimator& target) {
  (void)in;
  (void)target;
}

void SerializeCBTFImporter(YAML::Emitter& out, const CBTFImporter& target) {
  out << YAML::Key << "current_export_folder" << YAML::Value << target.m_currentExportFolder.string();
  out << YAML::Key << "import_folders" << YAML::Value << YAML::BeginSeq;
  for (const auto& folder : target.m_importFolders) {
    out << folder.string();
  }
  out << YAML::EndSeq;
}

void DeserializeCBTFImporter(const YAML::Node& in, CBTFImporter& target) {
  target.m_processing = false;
  if (in["current_export_folder"]) {
    target.m_currentExportFolder = in["current_export_folder"].as<std::string>();
  }
  target.m_importFolders.clear();
  if (in["import_folders"]) {
    for (const auto& folder : in["import_folders"]) {
      target.m_importFolders.emplace_back(folder.as<std::string>());
    }
  }
}
#endif

void RegisterDigitalAgricultureSerializationHandlers(const std::string& owner_name) {
  Serialization::RegisterSerializationHandler<SorghumDescriptor>(
      SerializeSorghumDescriptor, DeserializeSorghumDescriptor, owner_name, "SorghumDescriptor");
  Serialization::RegisterSerializationHandler<Sorghum>(SerializeSorghum, DeserializeSorghum, owner_name, "Sorghum");
  Serialization::RegisterSerializationHandler<SorghumGrowthStages>(
      SerializeSorghumGrowthStages, DeserializeSorghumGrowthStages, owner_name, "SorghumGrowthStages");
  Serialization::RegisterSerializationHandler<SorghumState>(SerializeSorghumState, DeserializeSorghumState, owner_name,
                                                            "SorghumState");
  Serialization::RegisterSerializationHandler<SorghumGenerator>(SerializeSorghumGenerator, DeserializeSorghumGenerator,
                                                                owner_name, "SorghumGenerator");
  Serialization::RegisterSerializationHandler<SorghumField>(SerializeSorghumField, DeserializeSorghumField, owner_name,
                                                            "SorghumField");
  Serialization::RegisterSerializationHandler<CropDescriptor>(SerializeCropDescriptor, DeserializeCropDescriptor,
                                                              owner_name, "CropDescriptor");
  Serialization::RegisterSerializationHandler<SorghumFieldGrid>(SerializeSorghumFieldGrid,
                                                                DeserializeSorghumFieldGrid, owner_name,
                                                                "SorghumFieldGrid");
  Serialization::RegisterSerializationHandler<SorghumTraitDescriptor>(
      SerializeSorghumTraitDescriptor, DeserializeSorghumTraitDescriptor, owner_name, "SorghumTraitDescriptor");
#ifdef CUDA_MODULE_SERVICE
  Serialization::RegisterSerializationHandler<PARSensorGroup>(SerializePARSensorGroup, DeserializePARSensorGroup,
                                                              owner_name, "PARSensorGroup");
  Serialization::RegisterSerializationHandler<CBTFGroup>(SerializeCBTFGroup, DeserializeCBTFGroup, owner_name,
                                                         "CBTFGroup");
  Serialization::RegisterSerializationHandler<LeafIlluminationEstimator>(
      SerializeLeafIlluminationEstimator, DeserializeLeafIlluminationEstimator, owner_name,
      "LeafIlluminationEstimator");
  Serialization::RegisterSerializationHandler<CBTFImporter>(SerializeCBTFImporter, DeserializeCBTFImporter, owner_name,
                                                            "CBTFImporter");
#endif
  Serialization::RegisterSerializationHandler<SkyIlluminance>(SerializeSkyIlluminance, DeserializeSkyIlluminance,
                                                              owner_name, "SkyIlluminance");
  Serialization::RegisterSerializationHandler<SorghumCoordinates>(
      SerializeSorghumCoordinates, DeserializeSorghumCoordinates, owner_name, "SorghumCoordinates");
}

void RegisterDigitalAgricultureInspectors(const std::string& owner_name) {
  InspectorRegistry::GetInstance().RegisterInspector<SorghumDescriptor>(InspectSorghumDescriptor, owner_name,
                                                                        "SorghumDescriptor");
  InspectorRegistry::GetInstance().RegisterInspector<Sorghum>(InspectSorghum, owner_name, "Sorghum");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumGrowthStages>(InspectSorghumGrowthStages, owner_name,
                                                                          "SorghumGrowthStages");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumState>(InspectSorghumState, owner_name, "SorghumState");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumGenerator>(InspectSorghumGenerator, owner_name,
                                                                       "SorghumGenerator");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumField>(InspectSorghumField, owner_name, "SorghumField");
  RegisterDrawGuiInspector<CropDescriptor>(owner_name, "CropDescriptor");
  RegisterDrawGuiInspector<SorghumFieldGrid>(owner_name, "SorghumFieldGrid");
  RegisterDrawGuiInspector<SorghumTraitDescriptor>(owner_name, "SorghumTraitDescriptor");
#ifdef CUDA_MODULE_SERVICE
  InspectorRegistry::GetInstance().RegisterInspector<PARSensorGroup>(InspectPARSensorGroup, owner_name,
                                                                     "PARSensorGroup");
  InspectorRegistry::GetInstance().RegisterInspector<CBTFGroup>(InspectCBTFGroup, owner_name, "CBTFGroup");
  InspectorRegistry::GetInstance().RegisterInspector<CBTFImporter>(InspectCBTFImporter, owner_name, "CBTFImporter");
  RegisterDrawGuiInspector<LeafIlluminationEstimator>(owner_name, "LeafIlluminationEstimator");
#endif
  InspectorRegistry::GetInstance().RegisterInspector<SkyIlluminance>(InspectSkyIlluminance, owner_name,
                                                                     "SkyIlluminance");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumCoordinates>(InspectSorghumCoordinates, owner_name,
                                                                         "SorghumCoordinates");
  InspectorRegistry::GetInstance().RegisterInspector<SorghumLayer>(InspectSorghumLayer, owner_name, "Sorghum Layer");
}
}  // namespace

EVOENGINE_PACKAGE_EXPORT const PackageDescriptor* EvoEnginePackageGetDescriptor() {
  return &descriptor;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageRegisterTypes(PackageRegistrar* registrar) {
  if (!registrar) {
    return false;
  }

  bool registered = registrar->RegisterAsset<SorghumDescriptor>("SorghumDescriptor", {".sorghum"}) &&
                    registrar->RegisterPrivateComponent<Sorghum>("Sorghum") &&
                    registrar->RegisterAsset<SorghumGrowthStages>("SorghumGrowthStages", {".sgs"}) &&
                    registrar->RegisterAsset<SorghumState>("SorghumState", {".ss"}) &&
                    registrar->RegisterAsset<SorghumGenerator>("SorghumGenerator", {".sg"}) &&
                    registrar->RegisterAsset<SorghumField>("SorghumField", {".sorghumfield"}) &&
                    registrar->RegisterAsset<CropDescriptor>("CropDescriptor", {".cropdesc"}) &&
                    registrar->RegisterAsset<SorghumTraitDescriptor>("SorghumTraitDescriptor", {".st"}) &&
                    registrar->RegisterPrivateComponent<SorghumFieldGrid>("SorghumFieldGrid");
#ifdef CUDA_MODULE_SERVICE
  registered = registered && registrar->RegisterAsset<PARSensorGroup>("PARSensorGroup", {".parsensorgroup"}) &&
               registrar->RegisterAsset<CBTFGroup>("CBTFGroup", {".cbtfgroup"}) &&
               registrar->RegisterPrivateComponent<CBTFImporter>("CBTFImporter") &&
               registrar->RegisterPrivateComponent<LeafIlluminationEstimator>("LeafIlluminationEstimator");
#endif
  registered = registered && registrar->RegisterAsset<SkyIlluminance>("SkyIlluminance", {".skyilluminance"}) &&
               registrar->RegisterAsset<SorghumCoordinates>("SorghumCoordinates", {".sorghumcoords"}) &&
               registrar->RegisterLayer<SorghumLayer>("Sorghum Layer");
  if (registered) {
    RegisterDigitalAgricultureSerializationHandlers(descriptor.name);
    RegisterDigitalAgricultureAssetPreviewHandlers(descriptor.name);
    RegisterDigitalAgricultureInspectors(descriptor.name);
  }
  return registered;
}

EVOENGINE_PACKAGE_EXPORT bool EvoEnginePackageLoad(PackageRegistrar*) {
  return true;
}

EVOENGINE_PACKAGE_EXPORT void EvoEnginePackageUnload(PackageRegistrar*) {
}
