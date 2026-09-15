file(GLOB_RECURSE EVOENGINE_EDITOR_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")
file(GLOB_RECURSE EVOENGINE_EDITOR_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(EvoEngine_EditorSDK ${EVOENGINE_SDK_LIBRARY_TYPE}
  ${EVOENGINE_EDITOR_HEADERS} ${EVOENGINE_EDITOR_SOURCES} ${IMGUI_SOURCES})
# The backend loads its own dispatch table through Platform, including across DLL boundaries.
set(EVOENGINE_IMGUI_VULKAN_BACKEND ${IMGUI_SOURCES})
list(FILTER EVOENGINE_IMGUI_VULKAN_BACKEND INCLUDE REGEX "/imgui_impl_vulkan[.]cpp$")
set_source_files_properties(${EVOENGINE_IMGUI_VULKAN_BACKEND} PROPERTIES
  SKIP_PRECOMPILE_HEADERS ON
  COMPILE_DEFINITIONS IMGUI_IMPL_VULKAN_NO_PROTOTYPES)
target_link_libraries(EvoEngine_EditorSDK PUBLIC EvoEngine_SDK)
target_include_directories(EvoEngine_EditorSDK PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" ${IMGUI_INCLUDES})
target_compile_definitions(EvoEngine_EditorSDK PRIVATE EVOENGINE_EDITOR_SDK_EXPORTS ${EVOENGINE_SDK_INTERNALS_DEF})
if(WIN32 AND EVOENGINE_ENABLE_RUNTIME_PACKAGES)
  target_compile_definitions(EvoEngine_EditorSDK PRIVATE "IMGUI_API=__declspec(dllexport)" "IGFD_API=__declspec(dllexport)")
  target_compile_definitions(EvoEngine_EditorSDK INTERFACE "IMGUI_API=__declspec(dllimport)" "IGFD_API=__declspec(dllimport)")
endif()
target_compile_options(EvoEngine_EditorSDK PRIVATE $<TARGET_PROPERTY:EvoEngine_SDK,COMPILE_OPTIONS>)
set(EVOENGINE_EDITOR_PCH "${CMAKE_CURRENT_SOURCE_DIR}/include/EvoEngine_Editor_PCH.hpp")
target_precompile_headers(EvoEngine_EditorSDK PRIVATE ${EVOENGINE_EDITOR_PCH})
set(EVOENGINE_EDITOR_PCH ${EVOENGINE_EDITOR_PCH} PARENT_SCOPE)
