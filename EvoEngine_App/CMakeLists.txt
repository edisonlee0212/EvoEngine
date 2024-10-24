message("=========Checking Apps=========")

add_custom_target(ResourceCopy)
evoengine_copy_resources(ResourceCopy ${CMAKE_CURRENT_BINARY_DIR})
evoengine_clear_shader_binaries(ResourceCopy)
add_custom_command(TARGET ResourceCopy POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
			${CMAKE_CURRENT_SOURCE_DIR}/imgui.ini ${CMAKE_CURRENT_BINARY_DIR} VERBATIM)

set_property(TARGET ResourceCopy PROPERTY FOLDER PostBuildActions)

function(register_evoengine_app app_name default_enable)
	if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
		set(option_name EvoEngine_App-${app_name})
		option(option_name "Build ${app_name} application" default_enable)
		if(option_name)
			add_executable(${app_name}
				${app_name}.cpp
				)
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
				${EvoEngine_Plugins}
				)
			add_dependencies(${app_name} ResourceCopy)
			set_property(TARGET ${app_name} PROPERTY FOLDER Executables)
			message("Enabled [${app_name}] application.")
		endif ()
	endif ()
endfunction()

register_evoengine_app(DemoApp ON)
register_evoengine_app(EcoSysLabApp ON)
register_evoengine_app(DigitalAgricultureApp ON)
register_evoengine_app(LogGraderApp ON)

register_evoengine_app(TreeDataGeneratorApp ON)
register_evoengine_app(SorghumDataGeneratorApp ON)
