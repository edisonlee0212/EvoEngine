#include "BillboardCloudsConverter.hpp"
#include "BillboardCloudsInspectionAdapters.hpp"
#include "EditorPackage.hpp"

using namespace evo_engine;
using namespace billboard_clouds_package;

EVOENGINE_PACKAGE_EXPORT const EditorPackageDescriptor* EvoEngineEditorPackageGetDescriptor() {
  static const EditorPackageDescriptor descriptor{
      EVOENGINE_EDITOR_PACKAGE_API_VERSION, "BillboardClouds",
      EVOENGINE_PACKAGE_SOURCE_ID,          EVOENGINE_PACKAGE_BUILD_IDENTITY,
      EVOENGINE_PACKAGE_RUNTIME_DESCRIPTOR, EVOENGINE_EDITOR_SOURCE_ID,
      EVOENGINE_EDITOR_PACKAGE_SOURCE_ID};
  return &descriptor;
}
EVOENGINE_PACKAGE_EXPORT bool EvoEngineEditorPackageLoad(EditorPackageRegistrar* registrar) {
  return registrar && registrar->RegisterInspector<BillboardCloudsConverter>(
                          [state = std::make_shared<BillboardCloudsInspectorState>()](
                              InspectorContext& context, BillboardCloudsConverter& target) {
                            return InspectBillboardCloudsConverter(context, target, *state);
                          },
                          "BillboardCloudsConverter");
}
EVOENGINE_PACKAGE_EXPORT void EvoEngineEditorPackageUnload() {
}
