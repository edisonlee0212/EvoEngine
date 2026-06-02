cmake_minimum_required(VERSION 3.17)

project(
	EvoEngine 
	VERSION 0.1 
	DESCRIPTION "An interactive application framework"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (MSVC)
	add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/FS>)
endif()

include(GNUInstallDirs)
include(CTest)

set(EVOENGINE_INSTALL_APP_RUNTIME_DIR ${CMAKE_INSTALL_BINDIR} CACHE STRING
	"Install directory for EvoEngine executable app runtimes.")
set(EVOENGINE_INSTALL_PYTHON_RUNTIME_DIR python CACHE STRING
	"Install directory for EvoEngine Python binding runtimes.")
option(EVOENGINE_INSTALL_PDBS "Install available MSVC PDB files with EvoEngine runtimes." ON)
option(EVOENGINE_ENABLE_RUNTIME_PACKAGES "Build EvoEngine runtime package support and package targets." ON)

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

if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease" OR CMAKE_BUILD_TYPE MATCHES "^[Rr]elWithDebInfo")
	set(DEBUG_STATUS NDEBUG)
elseif (CMAKE_BUILD_TYPE MATCHES "[Dd]ebug")
	set(DEBUG_STATUS _DEBUG)
endif ()

# RenderDoc Support
if(EXISTS "C:\\Program Files\\RenderDoc\\renderdoc_app.h")
add_compile_definitions(USE_RENDERDOC)
endif()

# We need a CMAKE_DIR with some code to find external dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# ------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------

set(3RDPARTY_INCLUDES "")
set(3RDPARTY_LIBS "")
set(3RDPARTY_DEFS "")
set(3RDPARTY_RUNTIME_LIBRARIES "")
set(3RDPARTY_PDBS "")
set(3RDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Extern/3rdParty)
add_subdirectory(${3RDPARTY_DIR})

# ------------------------------------------------------------------
# EvoEngine - SDK
# ------------------------------------------------------------------

set(EVOENGINE_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EvoEngine_SDK)
add_subdirectory(${EVOENGINE_SDK_DIR})

# ------------------------------------------------------------------
# EvoEngine - Services
# ------------------------------------------------------------------
set(SERVICES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EvoEngine_Services)
add_subdirectory(${SERVICES_DIR})

set(EVOENGINE_DEFS ${EVOENGINE_SDK_DEFS} ${EVOENGINE_SERVICES_DEFS})
set(EVOENGINE_INCLUDES ${EVOENGINE_SDK_INCLUDES} ${EVOENGINE_SERVICES_INCLUDES})
set(EVOENGINE_PCHS ${EVOENGINE_SDK_PCH} ${EVOENGINE_SERVICES_PCHS})
set(EVOENGINE_RUNTIME_LIBRARIES ${EVOENGINE_SDK_RUNTIME_LIBRARIES} ${3RDPARTY_RUNTIME_LIBRARIES} ${EVOENGINE_SERVICES_RUNTIME_LIBRARIES})
set(EVOENGINE_PDBS ${EVOENGINE_SDK_PDBS} ${3RDPARTY_PDBS} ${EVOENGINE_SERVICES_PDBS})

set(EVOENGINE_RESOURCES ${EVOENGINE_SDK_RESOURCES} ${EVOENGINE_SERVICES_RESOURCES})

# ------------------------------------------------------------------
# EvoEngine SDK - Copy resources
# ------------------------------------------------------------------
function(evoengine_copy_resources dst_target dst_dir)
	foreach (file_i ${EVOENGINE_RUNTIME_LIBRARIES})
		add_custom_command(TARGET ${dst_target} POST_BUILD
			COMMAND ${CMAKE_COMMAND}
				"-DSOURCE=${file_i}"
				"-DDESTINATION=${dst_dir}"
				-P ${CMAKE_SOURCE_DIR}/cmake/EvoEngineCopyIfExists.cmake
			VERBATIM)
	endforeach(file_i)

	foreach(file_i ${EVOENGINE_PDBS})
		add_custom_command(TARGET ${dst_target} POST_BUILD
			COMMAND ${CMAKE_COMMAND}
				"-DSOURCE=${file_i}"
				"-DDESTINATION=${dst_dir}"
				-P ${CMAKE_SOURCE_DIR}/cmake/EvoEngineCopyIfExists.cmake
			VERBATIM)
	endforeach(file_i)
	evoengine_sdk_copy_resources(${dst_target} ${dst_dir})
	foreach(folder_i ${EVOENGINE_RESOURCES})
		add_custom_command(TARGET ${dst_target} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory
			${folder_i} ${dst_dir}
			VERBATIM)
	endforeach(folder_i)

endfunction()
function(evoengine_clear_shader_binaries dst_target)
	set(dst_dir ${CMAKE_CURRENT_BINARY_DIR})
	if(ARGC GREATER 1)
		set(dst_dir ${ARGV1})
	endif()
	add_custom_command(TARGET ${dst_target} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E remove_directory ${dst_dir}/ShaderBinaries
		VERBATIM)
endfunction()

function(evoengine_install_optional_files dst_dir)
	foreach(file_i IN LISTS ARGN)
		if (file_i)
			install(FILES "${file_i}"
				DESTINATION "${dst_dir}"
				OPTIONAL)
		endif()
	endforeach()
endfunction()

function(evoengine_install_runtime_files dst_dir)
	evoengine_install_optional_files("${dst_dir}" ${EVOENGINE_RUNTIME_LIBRARIES})
	if (EVOENGINE_INSTALL_PDBS)
		evoengine_install_optional_files("${dst_dir}" ${EVOENGINE_PDBS})
	endif()
endfunction()

function(evoengine_install_runtime_resources dst_dir)
	install(DIRECTORY "${EVOENGINE_SDK_DIR}/Internals/"
		DESTINATION "${dst_dir}")
	foreach(folder_i IN LISTS EVOENGINE_RESOURCES)
		if (EXISTS "${folder_i}")
			install(DIRECTORY "${folder_i}/"
				DESTINATION "${dst_dir}")
		endif()
	endforeach()
endfunction()

function(evoengine_install_runtime_payload dst_dir)
	evoengine_install_runtime_files("${dst_dir}")
	evoengine_install_runtime_resources("${dst_dir}")
endfunction()

function(evoengine_install_target_pdb target_name dst_dir)
	if (MSVC AND EVOENGINE_INSTALL_PDBS)
		install(FILES "$<TARGET_PDB_FILE:${target_name}>"
			DESTINATION "${dst_dir}"
			OPTIONAL)
	endif()
endfunction()

# ------------------------------------------------------------------
# EvoEngine - Runtime Packages
# ------------------------------------------------------------------
if (EVOENGINE_ENABLE_RUNTIME_PACKAGES)
	set(PACKAGES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EvoEngine_Packages)
	add_subdirectory(${PACKAGES_DIR})
endif()

# ------------------------------------------------------------------
# EvoEngine - Apps
# ------------------------------------------------------------------
set(EVOENGINE_APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EvoEngine_App)
add_subdirectory(${EVOENGINE_APP_DIR})

# ------------------------------------------------------------------
# EvoEngine - Python Bindings
# ------------------------------------------------------------------
set(EVOENGINE_PYBIND_DIR ${CMAKE_CURRENT_SOURCE_DIR}/PythonBinding)
add_subdirectory(${EVOENGINE_PYBIND_DIR})

# ------------------------------------------------------------------
# EvoEngine - Tests
# ------------------------------------------------------------------
if (BUILD_TESTING)
	set(EVOENGINE_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EvoEngine_Tests)
	add_subdirectory(${EVOENGINE_TESTS_DIR})
endif()

if (NOT ${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	set(EVOENGINE_INCLUDES ${EVOENGINE_INCLUDES} PARENT_SCOPE)
	set(EVOENGINE_PCHS ${EVOENGINE_PCHS} PARENT_SCOPE)
	set(EVOENGINE_DEFS ${EVOENGINE_DEFS} PARENT_SCOPE)
	set(EVOENGINE_RUNTIME_LIBRARIES ${EVOENGINE_RUNTIME_LIBRARIES} PARENT_SCOPE)
	set(EVOENGINE_SDK_DIR ${EVOENGINE_SDK_DIR} PARENT_SCOPE)

	set(EVOENGINE_RESOURCES ${EVOENGINE_RESOURCES} PARENT_SCOPE)

	set(EvoEngine_Services ${EvoEngine_Services} PARENT_SCOPE)
endif ()

message("===============================")
message("|  CMake Configuration Done!  |")
message("===============================")
