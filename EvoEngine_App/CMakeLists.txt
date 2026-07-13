message("=========Checking Apps=========")

add_custom_target(AppResourceCopy)
add_dependencies(AppResourceCopy EvoEngine_SDK)

string (FIND "${EVOENGINE_DEFS}" "CUDA" CUDA_SUBSTRING_INDEX)
if(NOT CUDA_SUBSTRING_INDEX EQUAL -1)
    mark_as_advanced(CUDA_SDK_ROOT_DIR)
	enable_language(CUDA)
	message("CUDA enabled.")
endif()

set(EVOENGINE_APP_BUILD_RUNTIME_DIR ${CMAKE_CURRENT_BINARY_DIR})
if(CMAKE_CONFIGURATION_TYPES)
	set(EVOENGINE_APP_BUILD_RUNTIME_DIR ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
endif()
set(EVOENGINE_APP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(EVOENGINE_APP_RESOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources)

evoengine_copy_resources(AppResourceCopy ${EVOENGINE_APP_BUILD_RUNTIME_DIR})
evoengine_clear_shader_binaries(AppResourceCopy ${EVOENGINE_APP_BUILD_RUNTIME_DIR})
add_custom_command(TARGET AppResourceCopy POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
			${EVOENGINE_APP_RESOURCE_DIR}/imgui.ini ${EVOENGINE_APP_BUILD_RUNTIME_DIR} VERBATIM)

set_property(TARGET AppResourceCopy PROPERTY FOLDER PostBuildActions)

add_library(EvoEngine_DemoScene STATIC
	${CMAKE_CURRENT_SOURCE_DIR}/include/DemoScene.hpp
	${EVOENGINE_APP_SOURCE_DIR}/DemoScene.cpp
	)
target_compile_definitions(EvoEngine_DemoScene
	PRIVATE
	${DEFAULT_PROJECT_DEF}
	${EVOENGINE_DEFS}
	)
target_include_directories(EvoEngine_DemoScene
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${EVOENGINE_INCLUDES}
	)
target_precompile_headers(EvoEngine_DemoScene
	PRIVATE
	${EVOENGINE_PCHS}
	)
target_link_libraries(EvoEngine_DemoScene
	PUBLIC
	EvoEngine_SDK
	${EvoEngine_Services}
	)
if (EVOENGINE_ENABLE_RUNTIME_PACKAGES AND EvoEngine_RuntimePackages)
	add_dependencies(EvoEngine_DemoScene ${EvoEngine_RuntimePackages})
endif()
if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
	target_compile_options(EvoEngine_DemoScene PRIVATE -Werror)
elseif (MSVC)
	target_compile_options(EvoEngine_DemoScene PRIVATE /EHsc /W2 /FS /c)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL Clang)
	target_compile_options(EvoEngine_DemoScene PRIVATE -fPIC --no-warnings -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-newline-eof -Wno-padded -Wno-exit-time-destructors -Wno-global-constructors -Wno-constant-conversion)
endif ()
set_property(TARGET EvoEngine_DemoScene PROPERTY FOLDER Libraries)

function(register_evoengine_app app_name default_enable)
	if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
		set(option_name EvoEngine_App-${app_name})
		option(${option_name} "Build ${app_name} application" ${default_enable})
		if(${option_name})
			if (WIN32)
				add_executable(${app_name}
					${EVOENGINE_APP_SOURCE_DIR}/${app_name}.cpp
					${EVOENGINE_APP_RESOURCE_DIR}/${app_name}.rc
					)
				set_target_properties(${app_name} PROPERTIES WIN32_EXECUTABLE TRUE)
			else()
				add_executable(${app_name}
					${EVOENGINE_APP_SOURCE_DIR}/${app_name}.cpp
					)
			endif()
			if (${app_name} STREQUAL "EvoEngineLauncher")
				target_sources(${app_name}
					PRIVATE
					${EVOENGINE_APP_SOURCE_DIR}/LauncherUtils.cpp
					)
			endif()
			target_compile_definitions(${app_name}
				PRIVATE
				${DEFAULT_PROJECT_DEF}
				${EVOENGINE_DEFS}
				)
			target_include_directories(${app_name}
				PRIVATE
				${EVOENGINE_INCLUDES}
				${CMAKE_CURRENT_SOURCE_DIR}/include
				${EVOENGINE_APP_RESOURCE_DIR}
				)
			target_precompile_headers(${app_name}
				PRIVATE
				${EVOENGINE_PCHS}
				)
			target_link_libraries(${app_name}
				PRIVATE
				EvoEngine_SDK
				${EvoEngine_Services}
				)
			if (${app_name} STREQUAL "DemoApp")
				target_link_libraries(${app_name}
					PRIVATE
					EvoEngine_DemoScene
					)
			endif()
			if (${app_name} STREQUAL "DigitalAgricultureApp")
				target_link_libraries(${app_name}
					PRIVATE
					DigitalAgriculturePackage
					LSystemPackage
					)
				if (MSVC AND WIN32)
					target_link_libraries(${app_name}
						PRIVATE
						delayimp
						)
					target_link_options(${app_name}
						PRIVATE
						/DELAYLOAD:DigitalAgriculturePackage.dll
						/DELAYLOAD:LSystemPackage.dll
						)
				endif()
			endif()
			if (EVOENGINE_ENABLE_RUNTIME_PACKAGES AND EvoEngine_RuntimePackages)
				add_dependencies(${app_name} ${EvoEngine_RuntimePackages})
			endif()

			if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
				target_compile_options(${app_name} PRIVATE -Werror)
			elseif (MSVC)
				target_compile_options(${app_name} PRIVATE /EHsc /W2 /c)
				if (WIN32)
					target_link_options(${app_name} PRIVATE /ENTRY:mainCRTStartup)
				endif()
			elseif (CMAKE_CXX_COMPILER_ID STREQUAL Clang)
				target_compile_options(${app_name} PRIVATE -fPIC --no-warnings -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-newline-eof -Wno-padded -Wno-exit-time-destructors -Wno-global-constructors -Wno-constant-conversion)
			endif ()

			add_dependencies(${app_name} AppResourceCopy)
			install(TARGETS ${app_name}
				RUNTIME DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
			evoengine_install_target_pdb(${app_name} "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
			set_property(TARGET ${app_name} PROPERTY FOLDER Executables)
			message("Enabled [${app_name}] application.")
		endif ()
	endif ()
endfunction()

register_evoengine_app(EvoEngineEditor ON)
register_evoengine_app(EvoEngineLauncher ON)

register_evoengine_app(DemoApp ON)
register_evoengine_app(EcoSysLabApp OFF)
register_evoengine_app(DigitalAgricultureApp ON)
register_evoengine_app(LSystemApp ON)

if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	evoengine_install_runtime_payload("${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
	install(FILES ${EVOENGINE_APP_RESOURCE_DIR}/imgui.ini
		DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
endif ()
