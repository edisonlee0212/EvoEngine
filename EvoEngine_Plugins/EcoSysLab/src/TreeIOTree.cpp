/**
 * @author David Hrusa, Tomas Polasek
 * @date 11.20.2019
 * @version 1.0
 * @brief Simple module for representing importing and exporting skeleton files .tree (implements 1.0 .tree encoding).
 */

#include "TreeIOTree.hpp"

#include <csvpp.h>

namespace treeio {

void TreeNodeData::swapNodeCoords(int dira, int dirb) {
  float tmp;
  bool flip = (dira < 0) ? !(dirb < 0) : (dirb < 0);  // XOR
  float *tgta;
  float *tgtb;

  dira *= (dira < 0) ? -1 : 1;
  dirb *= (dirb < 0) ? -1 : 1;

  if (dira == 1) {
    tgta = &pos.x;
  } else if (dira == 2) {
    tgta = &pos.y;
  } else {
    tgta = &pos.z;
  }
  if (dirb == 1) {
    tgtb = &pos.x;
  } else if (dirb == 2) {
    tgtb = &pos.y;
  } else {
    tgtb = &pos.z;
  }

  tmp = *tgta * (flip ? -1.0f : 1.0f);
  *tgta = *tgtb * (flip ? -1.0f : 1.0f);
  *tgtb = tmp;
}

/// @brief Helper function for meta-data serialization.
void spliceValues(std::string &row1, std::string &row2, const std::string &txt1, const std::string &txt2) {
  row1 += "," + txt1;
  row2 += "," + txt2;
}

std::string TreeDynamicMetaData::serialize() const {
  return data.dump(4);
}

void TreeDynamicMetaData::deserialize(const std::string &serialized) {
  data = treeutil::containsOnlyWhiteSpaces(serialized) ? nlohmann::json{} : nlohmann::json::parse(serialized);
}

TreeMetaData::TreeMetaData() : mDynamicMetaData{std::make_shared<TreeDynamicMetaData>()} {
}

TreeMetaData::~TreeMetaData() { /* Automatic */
}

std::string TreeMetaData::serialize() const {
  // Push all runtime changes back to metaData proper:
  const_cast<TreeMetaData *>(this)->onSave();

  // Pseudo info (depends on the users):
  std::string row1 = "TreeID";
  std::string row2 = std::to_string(treeID);
  spliceValues(row1, row2, "Info", infoText);

  spliceValues(row1, row2, "Version", version);
  spliceValues(row1, row2, "TreeName", treeName);
  spliceValues(row1, row2, "TreeReference", treeReference);
  spliceValues(row1, row2, "Extension", extension);
  spliceValues(row1, row2, "Source", source);
  spliceValues(row1, row2, "ModelType", modelType);
  spliceValues(row1, row2, "Style", style);
  spliceValues(row1, row2, "Character", character);
  spliceValues(row1, row2, "ReqProcess", std::to_string(reqProcess));
  spliceValues(row1, row2, "Processed", std::to_string(processed));
  spliceValues(row1, row2, "Skeletonized", std::to_string(skeletonized));
  spliceValues(row1, row2, "Finalized", std::to_string(finalized));

  spliceValues(row1, row2, "Scale", std::to_string(skeletonScale));
  spliceValues(row1, row2, "ReferenceScale", std::to_string(referenceScale));
  spliceValues(row1, row2, "ReconstructionScale", std::to_string(reconstructionScale));
  spliceValues(row1, row2, "BaseScale", std::to_string(baseScale));
  spliceValues(row1, row2, "AgeEstimate", std::to_string(ageEstimate));
  spliceValues(row1, row2, "InternodalDistance", std::to_string(internodalDistance));

  spliceValues(row1, row2, "ThicknessFactor", std::to_string(thicknessFactor));
  spliceValues(row1, row2, "StartingThickness", std::to_string(startingThickness));
  spliceValues(row1, row2, "BranchTension", std::to_string(branchTension));
  spliceValues(row1, row2, "BranchBias", std::to_string(branchBias));
  spliceValues(row1, row2, "BranchWidthMultiplier", std::to_string(branchWidthMultiplier));
  spliceValues(row1, row2, "OldBranchWidthMultiplier", std::to_string(oldBranchWidthMultiplier));
  spliceValues(row1, row2, "RecalculateRadius", std::to_string(recalculateRadius));

  spliceValues(row1, row2, "DistinctAngle", std::to_string(distinctAngle));
  spliceValues(row1, row2, "Decimated", std::to_string(decimated));
  spliceValues(row1, row2, "DecimationEpsilon", std::to_string(decimationEpsilon));

  // Batch processing:
  spliceValues(row1, row2, "BatchScenario", batchScenario);

  // Prepare dynamic data block.
  std::stringstream dynamicBlock{};
  dynamicBlock << TreeDynamicMetaData::DYNAMIC_DATA_START_TAG << "\n"
               << (mDynamicMetaData ? mDynamicMetaData->serialize() : "") << "\n"
               << TreeDynamicMetaData::DYNAMIC_DATA_END_TAG << std::endl;

  // Prepare complete serialized data.
  std::stringstream serialized{};
  serialized << row1 << "\n" << row2 << "\n" << dynamicBlock.str();

  return serialized.str();
}

void TreeMetaData::deserialize(const std::string &serialized, const std::shared_ptr<TreeRuntimeMetaData> &runtime) {
  mDynamicMetaData = std::make_shared<TreeDynamicMetaData>();

  // Split data into static and dynamic parts
  const auto dynamicDataStartTagSize{std::string{TreeDynamicMetaData::DYNAMIC_DATA_START_TAG}.length()};
  const auto dynamicDataEndTagSize{std::string{TreeDynamicMetaData::DYNAMIC_DATA_END_TAG}.length()};
  const auto findIt{serialized.find(TreeDynamicMetaData::DYNAMIC_DATA_START_TAG)};
  const auto endIt{serialized.find(TreeDynamicMetaData::DYNAMIC_DATA_END_TAG)};
  const auto serializedStaticData{serialized.substr(0, findIt)};
  const auto dynamicDataStartIdx{findIt};
  const auto dynamicDataEndIdx{endIt + dynamicDataEndTagSize};
  const auto serializedDynamicDataTagged{
      (findIt != serialized.npos && endIt != serialized.npos)
          ? serialized.substr(dynamicDataStartIdx, dynamicDataEndIdx - dynamicDataStartIdx)
          : ""};
  const auto serializedDynamicData{
      serializedDynamicDataTagged.length() >= (dynamicDataStartTagSize + dynamicDataEndTagSize)
          ? serializedDynamicDataTagged.substr(
                dynamicDataStartTagSize,
                serializedDynamicDataTagged.size() - (dynamicDataStartTagSize + dynamicDataEndTagSize))
          : ""};

  // Deserialize static data.
  csvpp::RowReader reader;
  std::stringstream ss(serializedStaticData);
  ss >> reader;

  csvpp::rowiterator it;
  while (ss >> reader) {
    for (it = reader.begin(); it != reader.end(); it++) {
      insertValue(it->first, it->second);
    }
  }

  // Deserialize dynamic data.
  mDynamicMetaData->deserialize(serializedDynamicData);

  // Fix missing and invalid values.
  validateValues();

  // Once the deserialization is finished, refresh the runtime values.
  setRuntimeMetaData(runtime);
}

void TreeMetaData::insertValue(const std::string &key, const std::string &value) {
  const auto lowerKey{treeutil::strToLower(key)};

  // Pseudo info (depends on the users):
  if (!lowerKey.compare(treeutil::strToLower("TreeID"))) {
    treeID = atoi(value.c_str());
  } else if (!lowerKey.compare(treeutil::strToLower("Info"))) {
    infoText = value;
  } else if (!lowerKey.compare(treeutil::strToLower("BatchScenario"))) {
    batchScenario = value;
  }

  // Objective source / type info:
  else if (!lowerKey.compare(treeutil::strToLower("Version"))) {
    version = value;
  } else if (!lowerKey.compare(treeutil::strToLower("TreeName"))) {
    treeName = value;
  } else if (!lowerKey.compare(treeutil::strToLower("TreeReference"))) {
    treeReference = value;
  } else if (!lowerKey.compare(treeutil::strToLower("Extension"))) {
    extension = value;
  } else if (!lowerKey.compare(treeutil::strToLower("Source"))) {
    source = value;
  } else if (!lowerKey.compare(treeutil::strToLower("ModelType"))) {
    modelType = value;
  } else if (!lowerKey.compare(treeutil::strToLower("Style"))) {
    style = value;
  } else if (!lowerKey.compare(treeutil::strToLower("Character"))) {
    character = value;
  }

  else if (!lowerKey.compare(treeutil::strToLower("ReqProcess"))) {
    reqProcess = 0 != atoi(value.c_str());
  } else if (!lowerKey.compare(treeutil::strToLower("Processed"))) {
    processed = 0 != atoi(value.c_str());
  } else if (!lowerKey.compare(treeutil::strToLower("Skeletonized"))) {
    skeletonized = 0 != atoi(value.c_str());
  } else if (!lowerKey.compare(treeutil::strToLower("Finalized"))) {
    finalized = 0 != atoi(value.c_str());
  }

  else if (!lowerKey.compare(treeutil::strToLower("Scale"))) {
    skeletonScale = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("ReferenceScale"))) {
    referenceScale = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("ReconstructionScale"))) {
    reconstructionScale = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("BaseScale"))) {
    baseScale = static_cast<float>(atof(value.c_str()));
  }

  else if (!lowerKey.compare(treeutil::strToLower("AgeEstimate"))) {
    ageEstimate = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("InternodalDistance"))) {
    internodalDistance = static_cast<float>(atof(value.c_str()));
  }

  else if (!lowerKey.compare(treeutil::strToLower("ThicknessFactor"))) {
    thicknessFactor = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("StartingThickness"))) {
    startingThickness = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("BranchTension"))) {
    branchTension = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("BranchBias"))) {
    branchBias = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("BranchWidthMultiplier"))) {
    branchWidthMultiplier = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("OldBranchWidthMultiplier"))) {
    oldBranchWidthMultiplier = (value == "true");
  } else if (!lowerKey.compare(treeutil::strToLower("RecalculateRadius"))) {
    recalculateRadius = (value == "true");
  }

  else if (!lowerKey.compare(treeutil::strToLower("DistinctAngle"))) {
    distinctAngle = static_cast<float>(atof(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("Decimated"))) {
    decimated = 0 != static_cast<float>(atoi(value.c_str()));
  } else if (!lowerKey.compare(treeutil::strToLower("DecimationEpsilon"))) {
    decimationEpsilon = static_cast<float>(atof(value.c_str()));
  }
}

void TreeMetaData::validateValues() {
  // TODO - Create version conversion mechanism.
  // Fix missing scale values:
  if (referenceScale < 0.0f || (referenceScale == 1.0f && skeletonScale != 1.0f)) {
    referenceScale = skeletonScale;
  }
  if (reconstructionScale < 0.0f || (reconstructionScale == 1.0f && skeletonScale != 1.0f)) {
    reconstructionScale = skeletonScale;
  }
}

void TreeMetaData::onLoad() {
  if (mRuntimeMetaData) {
    mRuntimeMetaData->onLoad(*this);
  }
}

void TreeMetaData::onSave() {
  if (mRuntimeMetaData) {
    mRuntimeMetaData->onSave(*this);
  }
}

}  // namespace treeio
