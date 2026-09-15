#include "EditorPackage.hpp"
#include "TextureBaking.hpp"
#include "TextureBakingInspectionAdapters.hpp"

using namespace evo_engine;
using namespace texture_baking_package;

EVOENGINE_PACKAGE_EXPORT const EditorPackageDescriptor* EvoEngineEditorPackageGetDescriptor() {
  static const EditorPackageDescriptor descriptor{
      EVOENGINE_EDITOR_PACKAGE_API_VERSION, "TextureBaking",
      EVOENGINE_PACKAGE_SOURCE_ID,          EVOENGINE_PACKAGE_BUILD_IDENTITY,
      EVOENGINE_PACKAGE_RUNTIME_DESCRIPTOR, EVOENGINE_EDITOR_SOURCE_ID,
      EVOENGINE_EDITOR_PACKAGE_SOURCE_ID};
  return &descriptor;
}
EVOENGINE_PACKAGE_EXPORT bool EvoEngineEditorPackageLoad(EditorPackageRegistrar* registrar) {
  return registrar && registrar->RegisterInspector<TextureBaking>(InspectTextureBaking, "TextureBaking");
}
EVOENGINE_PACKAGE_EXPORT void EvoEngineEditorPackageUnload() {
}
