file(GLOB_RECURSE editor_sources CONFIGURE_DEPENDS "src/*.cpp" "include/*.hpp")
add_library(DigitalAgricultureEditorPackage SHARED ${editor_sources})
evoengine_configure_editor_package(DigitalAgricultureEditorPackage DigitalAgriculturePackage)
target_precompile_headers(DigitalAgricultureEditorPackage PRIVATE ${DIGITAL_AGRICULTURE_PCH})
