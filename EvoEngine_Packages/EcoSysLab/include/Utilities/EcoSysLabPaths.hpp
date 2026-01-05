#pragma once

#include <filesystem>

namespace eco_sys_lab_package {

/// Project-local `Metadata/` beside the `.eve` project (created on demand).
/// Falls back to `./Metadata` when no project is loaded.
std::filesystem::path EcoSysLabMetadataDirectory();

/// `EcoSysLabMetadataDirectory() / filename` (directory ensured).
std::filesystem::path EcoSysLabMetadataPath(const std::filesystem::path& filename);

}  // namespace eco_sys_lab_package
