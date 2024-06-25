if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR} AND EVOENGINE_PLUGIN_ECOSYSLAB)
	option(ECOSYSLAB_APPLICATION "Build EcoSysLab application" ON)
	if(ECOSYSLAB_APPLICATION)
		set(DEFAULT_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Resources/EcoSysLabProject/)
		set(DEFAULT_PROJECT_DEF ECOSYSLAB_DEFAULT_PROJECT_FOLDER=\"${DEFAULT_PROJECT_DIR}\")

		add_executable(EcoSysLabApp
			src/EcoSysLabApp.cpp)
		
		target_compile_definitions(EcoSysLabApp
			PRIVATE
			${DEFAULT_PROJECT_DEF}
			${EVOENGINE_DEFS}
			)
		target_include_directories(EcoSysLabApp
			PRIVATE
			${EVOENGINE_INCLUDES}
			)

		target_precompile_headers(EcoSysLabApp
			PRIVATE
			${EVOENGINE_PCHS}
			)

		target_link_libraries(EcoSysLabApp
			PRIVATE
			EvoEngine_SDK
			${EvoEngine_Plugins}
			)

		file(COPY imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		evoengine_copy_resources(EcoSysLabApp ${CMAKE_CURRENT_BINARY_DIR})
	endif ()
endif ()