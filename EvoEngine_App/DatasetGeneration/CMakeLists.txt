if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR} AND EVOENGINE_PLUGIN_ECOSYSLAB AND EVOENGINE_PLUGIN_DATASET_GENERATION)
	option(DATASET_GENERATOR_APPLICATION "Build DatasetGenerator application" ON)
	if(DATASET_GENERATOR_APPLICATION)
		set(DEFAULT_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Resources/EcoSysLabProject/)
		set(DEFAULT_PROJECT_DEF DATASET_GENERATOR_DEFAULT_PROJECT_FOLDER=\"${DEFAULT_PROJECT_DIR}\")

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
	endif ()
endif ()