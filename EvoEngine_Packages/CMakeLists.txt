message("====Checking Runtime Packages====")

set(EvoEngine_RuntimePackages "")
set(EVOENGINE_PACKAGE_PCH ${CMAKE_CURRENT_SOURCE_DIR}/include/EvoEngine_Package_PCH.hpp)

function(evoengine_configure_runtime_package target_name)
	set(multi_value_args EXTRA_PCH)
	cmake_parse_arguments(PACKAGE "" "" "${multi_value_args}" ${ARGN})

	target_precompile_headers(${target_name}
		PRIVATE
		${EVOENGINE_SDK_PCH}
		${EVOENGINE_PACKAGE_PCH}
		${PACKAGE_EXTRA_PCH}
		)
endfunction()

function(evoengine_discover_runtime_packages out_packages)
	file(GLOB package_cmake_files
		LIST_DIRECTORIES false
		CONFIGURE_DEPENDS
		"${CMAKE_CURRENT_SOURCE_DIR}/*/CMakeLists.txt")

	set(discovered_packages "")
	foreach(package_cmake_file ${package_cmake_files})
		get_filename_component(package_dir "${package_cmake_file}" DIRECTORY)
		get_filename_component(package_name "${package_dir}" NAME)
		if (NOT package_name MATCHES "^\\.")
			list(APPEND discovered_packages ${package_name})
		endif()
	endforeach()
	list(SORT discovered_packages)
	set(${out_packages} ${discovered_packages} PARENT_SCOPE)
endfunction()

function(evoengine_load_runtime_package_metadata package_name out_default_enable out_supported_platforms)
	set(default_enable ON)
	set(supported_platforms ALL)
	set(package_metadata_file "${CMAKE_CURRENT_SOURCE_DIR}/${package_name}/PackageInfo.cmake")

	if (EXISTS "${package_metadata_file}")
		set(EVOENGINE_PACKAGE_DEFAULT_ENABLE ${default_enable})
		set(EVOENGINE_PACKAGE_SUPPORTED_PLATFORMS ${supported_platforms})
		include("${package_metadata_file}")
		set(default_enable ${EVOENGINE_PACKAGE_DEFAULT_ENABLE})
		set(supported_platforms ${EVOENGINE_PACKAGE_SUPPORTED_PLATFORMS})
	endif()

	set(${out_default_enable} ${default_enable} PARENT_SCOPE)
	set(${out_supported_platforms} ${supported_platforms} PARENT_SCOPE)
endfunction()

function(evoengine_runtime_package_platform_supported supported_platforms out_supported)
	if (NOT supported_platforms OR supported_platforms STREQUAL "ALL")
		set(${out_supported} ON PARENT_SCOPE)
		return()
	endif()

	set(is_supported OFF)
	foreach(platform_name ${supported_platforms})
		string(TOUPPER "${platform_name}" platform_name_upper)
		if ((platform_name_upper STREQUAL "WINDOWS" OR platform_name_upper STREQUAL "WIN32") AND WIN32)
			set(is_supported ON)
		elseif ((platform_name_upper STREQUAL "LINUX" OR platform_name_upper STREQUAL "UNIX") AND UNIX AND NOT APPLE)
			set(is_supported ON)
		elseif ((platform_name_upper STREQUAL "MACOS" OR platform_name_upper STREQUAL "APPLE") AND APPLE)
			set(is_supported ON)
		endif()
	endforeach()

	set(${out_supported} ${is_supported} PARENT_SCOPE)
endfunction()

function(register_evoengine_runtime_package package_name)
	evoengine_load_runtime_package_metadata(${package_name} default_enable supported_platforms)
	set(option_name EVOENGINE_ENABLE_${package_name}_PACKAGE)
	option(${option_name} "Build ${package_name} runtime package." ${default_enable})

	evoengine_runtime_package_platform_supported("${supported_platforms}" package_platform_supported)
	if (NOT package_platform_supported)
		message("Skipping [${package_name}] runtime package: supported platforms are [${supported_platforms}].")
		return()
	endif()

	if (${option_name} AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${package_name})
		add_subdirectory(${package_name})
		if (TARGET ${package_name}Package)
			set(target_name ${package_name}Package)
			set_target_properties(${target_name} PROPERTIES
				FOLDER "Runtime Packages"
				RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/EvoEngine_App/Packages"
				LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/EvoEngine_App/Packages"
				PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/EvoEngine_App/Packages")
			foreach(config_name Debug Release RelWithDebInfo MinSizeRel)
				string(TOUPPER ${config_name} config_name_upper)
				set_target_properties(${target_name} PROPERTIES
					RUNTIME_OUTPUT_DIRECTORY_${config_name_upper} "${CMAKE_BINARY_DIR}/EvoEngine_App/${config_name}/Packages"
					LIBRARY_OUTPUT_DIRECTORY_${config_name_upper} "${CMAKE_BINARY_DIR}/EvoEngine_App/${config_name}/Packages"
					PDB_OUTPUT_DIRECTORY_${config_name_upper} "${CMAKE_BINARY_DIR}/EvoEngine_App/${config_name}/Packages")
			endforeach()
			install(TARGETS ${target_name}
				RUNTIME DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/Packages"
				LIBRARY DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/Packages")
			evoengine_install_target_pdb(${target_name} "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/Packages")
			set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} ${target_name} PARENT_SCOPE)
			message("Enabled [${package_name}] runtime package.")
		else()
			message("[${package_name}] runtime package not loaded.")
		endif()
	elseif (${option_name})
		message("[${package_name}] runtime package directory not found.")
	else()
		message("Disabled [${package_name}] runtime package.")
	endif()
endfunction()

evoengine_discover_runtime_packages(EVOENGINE_DISCOVERED_RUNTIME_PACKAGES)
foreach(package_name ${EVOENGINE_DISCOVERED_RUNTIME_PACKAGES})
	register_evoengine_runtime_package(${package_name})
endforeach()

set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} PARENT_SCOPE)
