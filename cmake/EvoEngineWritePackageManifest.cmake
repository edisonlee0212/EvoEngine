if (NOT DEFINED OUTPUT OR OUTPUT STREQUAL "")
	message(FATAL_ERROR "EvoEngineWritePackageManifest requires OUTPUT.")
endif()

if (NOT DEFINED NAME OR NAME STREQUAL "")
	message(FATAL_ERROR "EvoEngineWritePackageManifest requires NAME.")
endif()

if (NOT DEFINED LIBRARY OR LIBRARY STREQUAL "")
	message(FATAL_ERROR "EvoEngineWritePackageManifest requires LIBRARY.")
endif()

if (NOT DEFINED VERSION)
	set(VERSION "")
endif()

if (NOT DEFINED DESCRIPTION)
	set(DESCRIPTION "")
endif()

if (NOT DEFINED DEPENDENCIES)
	set(DEPENDENCIES "")
endif()

get_filename_component(output_dir "${OUTPUT}" DIRECTORY)
file(MAKE_DIRECTORY "${output_dir}")

file(WRITE "${OUTPUT}" "name: ${NAME}\n")
file(APPEND "${OUTPUT}" "library: ${LIBRARY}\n")
file(APPEND "${OUTPUT}" "version: ${VERSION}\n")
file(APPEND "${OUTPUT}" "description: ${DESCRIPTION}\n")
include("${IDENTITY_FILE}")
file(SHA256 "${LIBRARY_PATH}" library_sha256)
file(APPEND "${OUTPUT}"
	"sdk_source_id: ${EVOENGINE_SDK_SOURCE_ID}\n"
	"package_source_id: ${EVOENGINE_PACKAGE_SOURCE_ID_${NAME}}\n"
	"compiler_id: ${EVOENGINE_NATIVE_COMPILER_ID}\n"
	"compiler_version: ${EVOENGINE_NATIVE_COMPILER_VERSION}\n"
	"configuration: ${EVOENGINE_NATIVE_BUILD_CONFIGURATION}\n"
	"platform: ${EVOENGINE_NATIVE_PLATFORM}\n"
	"architecture: ${EVOENGINE_NATIVE_ARCHITECTURE}\n"
	"with_editor: ${EVOENGINE_NATIVE_WITH_EDITOR}\n"
	"library_sha256: ${library_sha256}\n")
file(APPEND "${OUTPUT}" "dependencies:\n")

if (DEPENDENCIES)
	string(REPLACE "|" ";" dependency_list "${DEPENDENCIES}")
	foreach(dependency_name IN LISTS dependency_list)
		if (NOT dependency_name STREQUAL "")
			file(APPEND "${OUTPUT}" "  - ${dependency_name}\n")
		endif()
	endforeach()
endif()

if(EDITOR_MANIFEST)
  file(APPEND "${OUTPUT}" "editor_manifest: ${EDITOR_MANIFEST}\n")
endif()
if(EDITOR_API_VERSION)
  file(APPEND "${OUTPUT}" "editor_source_id: ${EVOENGINE_EDITOR_SOURCE_ID}\n"
    "editor_package_source_id: ${EVOENGINE_EDITOR_PACKAGE_SOURCE_ID_${NAME}}\n")
  file(APPEND "${OUTPUT}" "editor_api_version: ${EDITOR_API_VERSION}\n")
endif()
