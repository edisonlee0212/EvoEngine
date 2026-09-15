#include "EditorPackage.hpp"
#include "UniverseEditorLayer.hpp"
#include "UniverseInspectionAdapters.hpp"
#include "UniverseLayer.hpp"
using namespace evo_engine;
using namespace universe_package;
EVOENGINE_PACKAGE_EXPORT const EditorPackageDescriptor* EvoEngineEditorPackageGetDescriptor() {
  static const EditorPackageDescriptor descriptor{
      EVOENGINE_EDITOR_PACKAGE_API_VERSION, "Universe",
      EVOENGINE_PACKAGE_SOURCE_ID,          EVOENGINE_PACKAGE_BUILD_IDENTITY,
      EVOENGINE_PACKAGE_RUNTIME_DESCRIPTOR, EVOENGINE_EDITOR_SOURCE_ID,
      EVOENGINE_EDITOR_PACKAGE_SOURCE_ID};
  return &descriptor;
}
EVOENGINE_PACKAGE_EXPORT bool EvoEngineEditorPackageLoad(EditorPackageRegistrar* registrar) {
  return registrar && registrar->RegisterLayer<UniverseEditorLayer>("Universe View") &&
         registrar->RegisterInspector<PlanetTerrain>(InspectPlanetTerrain, "PlanetTerrain") &&
         registrar->RegisterInspector<StarCluster>(InspectStarCluster, "Star Cluster") &&
         registrar->RegisterInspector<UniverseLayer>(InspectUniverseLayer, "Universe Layer");
}
EVOENGINE_PACKAGE_EXPORT void EvoEngineEditorPackageUnload() {
}
