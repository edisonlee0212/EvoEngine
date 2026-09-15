#include "EditorPackage.hpp"
#include "LSystemDescriptor.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemLayer.hpp"
#include "ScotsPine.hpp"
using namespace evo_engine;
using namespace l_system_package;
EVOENGINE_PACKAGE_EXPORT const EditorPackageDescriptor* EvoEngineEditorPackageGetDescriptor() {
  static const EditorPackageDescriptor descriptor{
      EVOENGINE_EDITOR_PACKAGE_API_VERSION, "LSystem",
      EVOENGINE_PACKAGE_SOURCE_ID,          EVOENGINE_PACKAGE_BUILD_IDENTITY,
      EVOENGINE_PACKAGE_RUNTIME_DESCRIPTOR, EVOENGINE_EDITOR_SOURCE_ID,
      EVOENGINE_EDITOR_PACKAGE_SOURCE_ID};
  return &descriptor;
}
EVOENGINE_PACKAGE_EXPORT bool EvoEngineEditorPackageLoad(EditorPackageRegistrar* registrar) {
  return registrar && registrar->RegisterInspector<LSystemDescriptor>(InspectLSystemDescriptor, "LSystemDescriptor") &&
         registrar->RegisterInspector<ScotsPine>(InspectScotsPine, "ScotsPine") &&
         registrar->RegisterInspector<LSystemLayer>(InspectLSystemLayer, "LSystem Layer") &&
         RegisterScotsPineDescriptorInspector(*registrar);
}
EVOENGINE_PACKAGE_EXPORT void EvoEngineEditorPackageUnload() {
}
