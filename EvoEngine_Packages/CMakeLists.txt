message("====Checking Runtime Packages====")

set(EvoEngine_RuntimePackages "")
set(EVOENGINE_PACKAGE_PCH ${CMAKE_CURRENT_SOURCE_DIR}/include/EvoEngine_Package_PCH.hpp)
set(EVOENGINE_PACKAGE_RESOURCES "")

set_property(GLOBAL PROPERTY EVOENGINE_REGISTERED_RUNTIME_PACKAGE_NAMES "")
set_property(GLOBAL PROPERTY EVOENGINE_REGISTERING_RUNTIME_PACKAGE_NAMES "")

function(evoengine_configure_runtime_package target_name)
	set(multi_value_args EXTRA_PCH RESOURCES)
	cmake_parse_arguments(PACKAGE "" "" "${multi_value_args}" ${ARGN})

	target_precompile_headers(${target_name}
		PRIVATE
		${EVOENGINE_SDK_PCH}
		${EVOENGINE_PACKAGE_PCH}
		${PACKAGE_EXTRA_PCH}
		)

	if (WIN32)
		set_target_properties(${target_name} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
	elseif (APPLE)
		set_target_properties(${target_name} PROPERTIES
			BUILD_RPATH "@loader_path"
			INSTALL_RPATH "@loader_path")
	else()
		set_target_properties(${target_name} PROPERTIES
			BUILD_RPATH "$ORIGIN"
			INSTALL_RPATH "$ORIGIN")
	endif()

	foreach(resource_dir IN LISTS PACKAGE_RESOURCES)
		if (EXISTS "${resource_dir}")
			get_filename_component(resource_folder_name "${resource_dir}" NAME)
			if (resource_folder_name STREQUAL "Internals")
				add_custom_command(TARGET ${target_name} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_directory
						"${resource_dir}" "$<TARGET_FILE_DIR:${target_name}>/.."
					VERBATIM)
				install(DIRECTORY "${resource_dir}/"
					DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
			else()
				add_custom_command(TARGET ${target_name} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_directory
						"${resource_dir}" "$<TARGET_FILE_DIR:${target_name}>/../${resource_folder_name}"
					VERBATIM)
				install(DIRECTORY "${resource_dir}/"
					DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/${resource_folder_name}")
			endif()
		endif()
	endforeach()

	if (DEFINED EVOENGINE_CURRENT_PACKAGE_NAME AND NOT EVOENGINE_CURRENT_PACKAGE_NAME STREQUAL "")
		evoengine_generate_runtime_package_manifest(
			${EVOENGINE_CURRENT_PACKAGE_NAME}
			${target_name}
			"${EVOENGINE_CURRENT_PACKAGE_VERSION}"
			"${EVOENGINE_CURRENT_PACKAGE_DESCRIPTION}"
			"${EVOENGINE_CURRENT_PACKAGE_DEPENDENCIES}")
	endif()
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

function(evoengine_load_runtime_package_metadata package_name out_default_enable out_supported_platforms out_dependencies out_version out_description)
	set(default_enable ON)
	set(supported_platforms ALL)
	set(dependencies "")
	set(version "0.1.0")
	set(description "${package_name} runtime package.")
	set(package_metadata_file "${CMAKE_CURRENT_SOURCE_DIR}/${package_name}/PackageInfo.cmake")

	if (EXISTS "${package_metadata_file}")
		set(EVOENGINE_PACKAGE_DEFAULT_ENABLE ${default_enable})
		set(EVOENGINE_PACKAGE_SUPPORTED_PLATFORMS ${supported_platforms})
		set(EVOENGINE_PACKAGE_DEPENDS ${dependencies})
		set(EVOENGINE_PACKAGE_VERSION ${version})
		set(EVOENGINE_PACKAGE_DESCRIPTION ${description})
		include("${package_metadata_file}")
		set(default_enable ${EVOENGINE_PACKAGE_DEFAULT_ENABLE})
		set(supported_platforms ${EVOENGINE_PACKAGE_SUPPORTED_PLATFORMS})
		set(dependencies ${EVOENGINE_PACKAGE_DEPENDS})
		set(version ${EVOENGINE_PACKAGE_VERSION})
		set(description ${EVOENGINE_PACKAGE_DESCRIPTION})
	endif()

	set(${out_default_enable} ${default_enable} PARENT_SCOPE)
	set(${out_supported_platforms} ${supported_platforms} PARENT_SCOPE)
	set(${out_dependencies} ${dependencies} PARENT_SCOPE)
	set(${out_version} ${version} PARENT_SCOPE)
	set(${out_description} ${description} PARENT_SCOPE)
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

function(evoengine_generate_runtime_package_manifest package_name target_name package_version package_description package_dependencies)
	string(REPLACE ";" "|" dependencies_encoded "${package_dependencies}")
	add_custom_command(TARGET ${target_name} POST_BUILD
		COMMAND ${CMAKE_COMMAND}
			"-DOUTPUT=$<TARGET_FILE_DIR:${target_name}>/${package_name}.evepackage"
			"-DNAME=${package_name}"
			"-DLIBRARY=$<TARGET_FILE_NAME:${target_name}>"
			"-DVERSION=${package_version}"
			"-DDESCRIPTION=${package_description}"
			"-DDEPENDENCIES=${dependencies_encoded}"
			-P "${CMAKE_SOURCE_DIR}/cmake/EvoEngineWritePackageManifest.cmake"
		VERBATIM)
	install(FILES "$<TARGET_FILE_DIR:${target_name}>/${package_name}.evepackage"
		DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/Packages"
		OPTIONAL)
endfunction()

function(register_evoengine_runtime_package package_name)
	get_property(registered_packages GLOBAL PROPERTY EVOENGINE_REGISTERED_RUNTIME_PACKAGE_NAMES)
	if (package_name IN_LIST registered_packages)
		return()
	endif()

	get_property(registering_packages GLOBAL PROPERTY EVOENGINE_REGISTERING_RUNTIME_PACKAGE_NAMES)
	if (package_name IN_LIST registering_packages)
		message(FATAL_ERROR "Runtime package dependency cycle detected while registering [${package_name}].")
	endif()
	set_property(GLOBAL APPEND PROPERTY EVOENGINE_REGISTERING_RUNTIME_PACKAGE_NAMES ${package_name})

	evoengine_load_runtime_package_metadata(${package_name} default_enable supported_platforms package_dependencies package_version package_description)
	set(option_name EVOENGINE_ENABLE_${package_name}_PACKAGE)
	option(${option_name} "Build ${package_name} runtime package." ${default_enable})

	evoengine_runtime_package_platform_supported("${supported_platforms}" package_platform_supported)
	if (NOT package_platform_supported)
		message("Skipping [${package_name}] runtime package: supported platforms are [${supported_platforms}].")
		set_property(GLOBAL APPEND PROPERTY EVOENGINE_REGISTERED_RUNTIME_PACKAGE_NAMES ${package_name})
		return()
	endif()

	if (${option_name})
		foreach(dependency_name ${package_dependencies})
			if (dependency_name IN_LIST EVOENGINE_DISCOVERED_RUNTIME_PACKAGES)
				register_evoengine_runtime_package(${dependency_name})
			endif()
			if (NOT TARGET ${dependency_name}Package)
				message("[${package_name}] runtime package not loaded: dependency [${dependency_name}] runtime package is not available.")
				set_property(GLOBAL APPEND PROPERTY EVOENGINE_REGISTERED_RUNTIME_PACKAGE_NAMES ${package_name})
				set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} PARENT_SCOPE)
				return()
			endif()
		endforeach()

		if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${package_name})
			set(EVOENGINE_CURRENT_PACKAGE_NAME ${package_name})
			set(EVOENGINE_CURRENT_PACKAGE_VERSION ${package_version})
			set(EVOENGINE_CURRENT_PACKAGE_DESCRIPTION ${package_description})
			set(EVOENGINE_CURRENT_PACKAGE_DEPENDENCIES ${package_dependencies})
			add_subdirectory(${package_name})
			unset(EVOENGINE_CURRENT_PACKAGE_NAME)
			unset(EVOENGINE_CURRENT_PACKAGE_VERSION)
			unset(EVOENGINE_CURRENT_PACKAGE_DESCRIPTION)
			unset(EVOENGINE_CURRENT_PACKAGE_DEPENDENCIES)
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
				set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} ${target_name})
				message("Enabled [${package_name}] runtime package.")
			else()
				message("[${package_name}] runtime package not loaded.")
			endif()
		else()
			message("[${package_name}] runtime package directory not found.")
		endif()
	else()
		message("Disabled [${package_name}] runtime package.")
	endif()

	set_property(GLOBAL APPEND PROPERTY EVOENGINE_REGISTERED_RUNTIME_PACKAGE_NAMES ${package_name})
	set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} PARENT_SCOPE)
endfunction()

evoengine_discover_runtime_packages(EVOENGINE_DISCOVERED_RUNTIME_PACKAGES)
foreach(package_name ${EVOENGINE_DISCOVERED_RUNTIME_PACKAGES})
	register_evoengine_runtime_package(${package_name})
endforeach()

set(EvoEngine_RuntimePackages ${EvoEngine_RuntimePackages} PARENT_SCOPE)
