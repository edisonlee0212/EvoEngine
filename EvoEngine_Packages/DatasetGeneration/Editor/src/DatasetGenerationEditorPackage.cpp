#include "DatasetGenerationInspectionAdapters.hpp"
#include "EditorPackage.hpp"
using namespace evo_engine;
using namespace dataset_generation_package;
EVOENGINE_PACKAGE_EXPORT const EditorPackageDescriptor* EvoEngineEditorPackageGetDescriptor() {
  static const EditorPackageDescriptor descriptor{
      EVOENGINE_EDITOR_PACKAGE_API_VERSION, "DatasetGeneration",
      EVOENGINE_PACKAGE_SOURCE_ID,          EVOENGINE_PACKAGE_BUILD_IDENTITY,
      EVOENGINE_PACKAGE_RUNTIME_DESCRIPTOR, EVOENGINE_EDITOR_SOURCE_ID,
      EVOENGINE_EDITOR_PACKAGE_SOURCE_ID};
  return &descriptor;
}
EVOENGINE_PACKAGE_EXPORT bool EvoEngineEditorPackageLoad(EditorPackageRegistrar* registrar) {
  return registrar &&
         registrar->RegisterInspector<TreePointCloudScanner>(
             [state = std::make_shared<TreePointCloudScannerInspector>()](InspectorContext& context,
                                                                          TreePointCloudScanner& scanner) {
               return state->Inspect(context, scanner);
             },
             "TreePointCloudScanner") &&
         registrar->RegisterInspector<SorghumPointCloudScanner>(
             [state = std::make_shared<SorghumPointCloudScannerInspector>()](InspectorContext& context,
                                                                             SorghumPointCloudScanner& scanner) {
               return state->Inspect(context, scanner);
             },
             "SorghumPointCloudScanner");
}
EVOENGINE_PACKAGE_EXPORT void EvoEngineEditorPackageUnload() {
}
