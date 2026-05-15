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
file(APPEND "${OUTPUT}" "dependencies:\n")

if (DEPENDENCIES)
	string(REPLACE "|" ";" dependency_list "${DEPENDENCIES}")
	foreach(dependency_name IN LISTS dependency_list)
		if (NOT dependency_name STREQUAL "")
			file(APPEND "${OUTPUT}" "  - ${dependency_name}\n")
		endif()
	endforeach()
endif()
