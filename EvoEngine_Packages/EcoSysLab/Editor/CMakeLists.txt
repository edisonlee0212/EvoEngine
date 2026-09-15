file(GLOB_RECURSE editor_sources CONFIGURE_DEPENDS "src/*.cpp" "include/*.hpp")
add_library(EcoSysLabEditorPackage SHARED ${editor_sources})
evoengine_configure_editor_package(EcoSysLabEditorPackage EcoSysLabPackage DEPENDS BillboardClouds)
target_precompile_headers(EcoSysLabEditorPackage PRIVATE ${ECOSYSLAB_PCH})
