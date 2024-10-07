cmake_minimum_required(VERSION 3.17)

project(
	EvoEngine 
	VERSION 0.1 
	DESCRIPTION "An interactive application framework"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ------------------------------------------------------------------
# Build types
# ------------------------------------------------------------------
# Set a default build type if none was specified
set(default_build_type "RelWithDebInfo")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
			STRING "Choose the type of build." FORCE)
endif ()

if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease" OR CMAKE_BUILD_TYPE MATCHES "^[Rr]lWithDebInfo")
	set(DEBUG_STATUS NDEBUG)
elseif (CMAKE_BUILD_TYPE MATCHES "[Dd]ebug")
	set(DEBUG_STATUS _DEBUG)
endif ()

# We need a CMAKE_DIR with some code to find external dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# ------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------

set(3RDPARTY_INCLUDES "")
set(3RDPARTY_LIBS "")
set(3RDPARTY_DLLS "")
set(3RDPARTY_PDBS "")
set(3RDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Extern/3rdParty)
add_subdirectory(${3RDPARTY_DIR})

# ------------------------------------------------------------------
# EvoEngine - SDK
# ------------------------------------------------------------------

set(EVOENGINE_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EvoEngine_SDK)
add_subdirectory(${EVOENGINE_SDK_DIR})

# ------------------------------------------------------------------
# EvoEngine - Plugins
# ------------------------------------------------------------------
set(PLUGINS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EvoEngine_Plugins)
add_subdirectory(${PLUGINS_DIR})

set(EVOENGINE_DEFS ${EVOENGINE_SDK_DEFS} ${EVOENGINE_PLUGINS_DEFS})
set(EVOENGINE_INCLUDES ${EVOENGINE_SDK_INCLUDES} ${EVOENGINE_PLUGINS_INCLUDES})
set(EVOENGINE_PCHS ${EVOENGINE_SDK_PCH} ${EVOENGINE_PLUGINS_PCHS})
set(EVOENGINE_DLLS ${EVOENGINE_SDK_DLLS} ${3RDPARTY_DLLS} ${EVOENGINE_PLUGINS_DLLS})
set(EVOENGINE_PDBS ${EVOENGINE_SDK_PDBS} ${3RDPARTY_PDBS} ${EVOENGINE_PLUGINS_PDBS})

set(EVOENGINE_RESOURCES ${EVOENGINE_SDK_RESOURCES} ${EVOENGINE_PLUGINS_RESOURCES})

# ------------------------------------------------------------------
# EvoEngine SDK - Copy resources
# ------------------------------------------------------------------
function(evoengine_copy_resources dst_target dst_dir)
	if (WIN32)
		foreach (file_i ${EVOENGINE_DLLS})
			string(REPLACE "/" "\\" file_i ${file_i})
			add_custom_command(TARGET ${dst_target} POST_BUILD
				COMMAND cmd /c "if exist \"${file_i}\" copy \"${file_i}\" \"${dst_dir}\\\"")
		endforeach(file_i)

		foreach(file_i ${EVOENGINE_PDBS})
			string(REPLACE "/" "\\" file_i ${file_i})
			add_custom_command(TARGET ${dst_target} POST_BUILD
				COMMAND cmd /c "if exist \"${file_i}\" copy \"${file_i}\" \"${dst_dir}\\\"")
		endforeach(file_i)
	else()
		foreach (file_i ${EVOENGINE_DLLS})
			add_custom_command(TARGET ${dst_target} POST_BUILD
				/bin/sh -c "if [ -e \"${file_i}\" ]; then cp \"${file_i}\" \"${dst_dir}\\\"; fi")
		endforeach(file_i)

		foreach(file_i ${EVOENGINE_PDBS})
			add_custom_command(TARGET ${dst_target} POST_BUILD
				/bin/sh -c "if [ -e \"${file_i}\" ]; then cp \"${file_i}\" \"${dst_dir}\\\"; fi")
		endforeach(file_i)
	endif()
	evoengine_sdk_copy_resources(${dst_target} ${dst_dir})
	foreach(folder_i ${EVOENGINE_RESOURCES})
		add_custom_command(TARGET ${dst_target} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
			${folder_i} ${dst_dir}
			VERBATIM)
	endforeach(folder_i)

endfunction()
function(evoengine_clear_shader_binaries dst_target)
	add_custom_command(TARGET ${dst_target} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_BINARY_DIR}/ShaderBinaries
		VERBATIM)
endfunction()
# ------------------------------------------------------------------
# Demo
# ------------------------------------------------------------------

set(EVOENGINE_APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EvoEngine_App)
add_subdirectory(${EVOENGINE_APP_DIR})

# ------------------------------------------------------------------
# EvoEngine - python binding
# ------------------------------------------------------------------
#if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
#	file(GLOB LOCAL_PYEVOENGINE_SOURCES "src/python/*.cpp")
#	pybind11_add_module(pyevoengine ${LOCAL_PYEVOENGINE_SOURCES})
#	target_include_directories(pyevoengine
#				PRIVATE
#				${LOCAL_EVOENGINE_INCLUDES}
#				${CMAKE_CURRENT_SOURCE_DIR}/include/python
#				)
#	target_precompile_headers(pyevoengine
#				PRIVATE
#				${LOCAL_EVOENGINE_PCH}
#				)
#	target_link_libraries(pyevoengine
#				PRIVATE
#				evoengine
#				)
#endif ()

if (NOT ${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	set(EVOENGINE_INCLUDES ${EVOENGINE_INCLUDES} PARENT_SCOPE)
	set(EVOENGINE_PCHS ${EVOENGINE_PCHS} PARENT_SCOPE)
	set(EVOENGINE_DEFS ${EVOENGINE_DEFS} PARENT_SCOPE)
	set(EVOENGINE_DLLS ${EVOENGINE_DLLS} PARENT_SCOPE)
	set(EVOENGINE_SDK_DIR ${EVOENGINE_SDK_DIR} PARENT_SCOPE)

	set(EVOENGINE_RESOURCES ${EVOENGINE_RESOURCES} PARENT_SCOPE)

	set(EvoEngine_Plugins ${EvoEngine_Plugins} PARENT_SCOPE)
endif ()