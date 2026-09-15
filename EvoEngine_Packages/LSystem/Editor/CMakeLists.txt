file(GLOB_RECURSE editor_sources CONFIGURE_DEPENDS "src/*.cpp" "include/*.hpp")
add_library(LSystemEditorPackage SHARED ${editor_sources})
evoengine_configure_editor_package(LSystemEditorPackage LSystemPackage)
target_precompile_headers(LSystemEditorPackage PRIVATE ${LSYSTEM_PCH})
