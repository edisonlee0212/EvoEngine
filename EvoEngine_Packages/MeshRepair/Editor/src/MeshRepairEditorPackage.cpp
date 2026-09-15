#include "EditorPackage.hpp"
#include "MeshColoring.hpp"
#include "MeshRepairInspectionAdapters.hpp"

using namespace evo_engine;
using namespace mesh_repair_package;

EVOENGINE_PACKAGE_EXPORT const EditorPackageDescriptor* EvoEngineEditorPackageGetDescriptor() {
  static const EditorPackageDescriptor descriptor{
      EVOENGINE_EDITOR_PACKAGE_API_VERSION, "MeshRepair",
      EVOENGINE_PACKAGE_SOURCE_ID,          EVOENGINE_PACKAGE_BUILD_IDENTITY,
      EVOENGINE_PACKAGE_RUNTIME_DESCRIPTOR, EVOENGINE_EDITOR_SOURCE_ID,
      EVOENGINE_EDITOR_PACKAGE_SOURCE_ID};
  return &descriptor;
}
EVOENGINE_PACKAGE_EXPORT bool EvoEngineEditorPackageLoad(EditorPackageRegistrar* registrar) {
  return registrar &&
         registrar->RegisterInspector<MeshColoring>(
             [state = std::make_shared<MeshRepairInspectorState>()](InspectorContext& context, MeshColoring& target) {
               return InspectMeshColoring(context, target, *state);
             },
             "MeshColoring");
}
EVOENGINE_PACKAGE_EXPORT void EvoEngineEditorPackageUnload() {
}
