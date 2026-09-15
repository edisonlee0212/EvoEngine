file(GLOB_RECURSE editor_sources CONFIGURE_DEPENDS "src/*.cpp" "include/*.hpp")
add_library(MeshRepairEditorPackage SHARED ${editor_sources})
evoengine_configure_editor_package(MeshRepairEditorPackage MeshRepairPackage)
