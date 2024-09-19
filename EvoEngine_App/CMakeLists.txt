message("=========Checking Apps=========")

function(register_evoengine_app app_name default_enable)
	if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
		set(option_name EvoEngine_App-${app_name})
		option(option_name "Build ${app_name} application" ON)
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
				EvoEngine_SDK
				${EvoEngine_Plugins}
				)
			file(COPY imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
			evoengine_copy_resources(${app_name} ${CMAKE_CURRENT_BINARY_DIR})
			set_property(TARGET ${app_name} PROPERTY FOLDER "Apps")
		endif ()
	endif ()
endfunction()

register_evoengine_app(DemoApp ON)
register_evoengine_app(EcoSysLabApp ON)
register_evoengine_app(DigitalAgricultureApp ON)
register_evoengine_app(LogGraderApp ON)


if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR} AND EVOENGINE_PLUGIN_ECOSYSLAB AND EVOENGINE_PLUGIN_DATASET_GENERATION)
	option(DATASET_GENERATOR_APPLICATION "Build DatasetGenerator application" ON)
	if(DATASET_GENERATOR_APPLICATION)
		add_executable(DatasetGeneratorApp
			DatasetGeneratorApp.cpp)
		
		target_compile_definitions(DatasetGeneratorApp
			PRIVATE
			${DEFAULT_PROJECT_DEF}
			${EVOENGINE_DEFS}
			)
		target_include_directories(DatasetGeneratorApp
			PRIVATE
			${EVOENGINE_INCLUDES}
			)

		target_precompile_headers(DatasetGeneratorApp
			PRIVATE
			${EVOENGINE_PCHS}
			)

		target_link_libraries(DatasetGeneratorApp
			PRIVATE
			EvoEngine_SDK
			${EvoEngine_Plugins}
			)

		file(COPY imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		evoengine_copy_resources(DatasetGeneratorApp ${CMAKE_CURRENT_BINARY_DIR})
		set_property(TARGET DatasetGeneratorApp PROPERTY FOLDER "Apps")
	endif ()
endif ()







