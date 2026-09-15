file(GLOB_RECURSE editor_sources CONFIGURE_DEPENDS "src/*.cpp" "include/*.hpp")
add_library(DatasetGenerationEditorPackage SHARED ${editor_sources})
evoengine_configure_editor_package(DatasetGenerationEditorPackage DatasetGenerationPackage)
target_precompile_headers(DatasetGenerationEditorPackage PRIVATE ${DATASET_GENERATION_PCH})
