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

evoengine_copy_resources(AppResourceCopy ${EVOENGINE_APP_BUILD_RUNTIME_DIR})
evoengine_clear_shader_binaries(AppResourceCopy ${EVOENGINE_APP_BUILD_RUNTIME_DIR})
add_custom_command(TARGET AppResourceCopy POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
			${CMAKE_CURRENT_SOURCE_DIR}/imgui.ini ${EVOENGINE_APP_BUILD_RUNTIME_DIR} VERBATIM)

set_property(TARGET AppResourceCopy PROPERTY FOLDER PostBuildActions)

function(register_evoengine_app app_name default_enable)
	if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
		set(option_name EvoEngine_App-${app_name})
		option(${option_name} "Build ${app_name} application" ${default_enable})
		if(${option_name})
			if (WIN32)
				add_executable(${app_name}
					${app_name}.cpp
					${app_name}.rc
					)
			else()
				add_executable(${app_name}
					${app_name}.cpp
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
			if (EVOENGINE_ENABLE_RUNTIME_PACKAGES AND EvoEngine_RuntimePackages)
				add_dependencies(${app_name} ${EvoEngine_RuntimePackages})
			endif()

			if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
				target_compile_options(${app_name} PRIVATE -Werror)
			elseif (MSVC)
				target_compile_options(${app_name} PRIVATE /EHsc /W2 /c)
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

register_evoengine_app(DemoApp ON)
register_evoengine_app(EcoSysLabApp ON)
register_evoengine_app(DigitalAgricultureApp ON)
register_evoengine_app(LogGradingApp ON)

register_evoengine_app(TreeDataGeneratorApp ON)
register_evoengine_app(SorghumDataGeneratorApp ON)




# Setup an empty application for quicker compilation for testing SDK
if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	set(option_name EvoEngine_App-EmptyApp)
	option(${option_name} "Build Empty application" ON)
	if(${option_name})
		add_executable(EmptyApp
			EmptyApp.cpp
			)
		target_compile_definitions(EmptyApp
			PRIVATE
			${DEFAULT_PROJECT_DEF}
			${EVOENGINE_DEFS}
			)
		target_include_directories(EmptyApp
			PRIVATE
			${EVOENGINE_INCLUDES}
			)
		target_precompile_headers(EmptyApp
			PRIVATE
			${EVOENGINE_PCHS}
			)
		target_link_libraries(EmptyApp
			PRIVATE
			EvoEngine_SDK
		)
		if (EVOENGINE_ENABLE_RUNTIME_PACKAGES AND EvoEngine_RuntimePackages)
			add_dependencies(EmptyApp ${EvoEngine_RuntimePackages})
		endif()

		if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
			target_compile_options(EmptyApp PRIVATE -Werror)
		elseif (MSVC)
			target_compile_options(EmptyApp PRIVATE /EHsc /W2 /c)
		elseif (CMAKE_CXX_COMPILER_ID STREQUAL Clang)
			target_compile_options(EmptyApp PRIVATE -fPIC --no-warnings -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-newline-eof -Wno-padded -Wno-exit-time-destructors -Wno-global-constructors -Wno-constant-conversion)
		endif ()

		add_dependencies(EmptyApp AppResourceCopy)
		install(TARGETS EmptyApp
			RUNTIME DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
		evoengine_install_target_pdb(EmptyApp "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
		set_property(TARGET EmptyApp PROPERTY FOLDER Executables)
	endif ()
endif ()

if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	evoengine_install_runtime_payload("${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/imgui.ini
		DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}")
endif ()
