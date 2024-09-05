if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR} AND EVOENGINE_PLUGIN_DIGITAL_AGRICULTURE)
	option(ECOSYSLAB_APPLICATION "Build DigitalAgriculture application" ON)
	if(ECOSYSLAB_APPLICATION)
		set(DEFAULT_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Resources/DigitalAgricultureProject/)
		set(DEFAULT_PROJECT_DEF ECOSYSLAB_DEFAULT_PROJECT_FOLDER=\"${DEFAULT_PROJECT_DIR}\")

		add_executable(DigitalAgricultureApp
			DigitalAgricultureApp.cpp)
		
		target_compile_definitions(DigitalAgricultureApp
			PRIVATE
			${DEFAULT_PROJECT_DEF}
			${EVOENGINE_DEFS}
			)
		target_include_directories(DigitalAgricultureApp
			PRIVATE
			${EVOENGINE_INCLUDES}
			)

		target_precompile_headers(DigitalAgricultureApp
			PRIVATE
			${EVOENGINE_PCHS}
			)

		target_link_libraries(DigitalAgricultureApp
			PRIVATE
			EvoEngine_SDK
			${EvoEngine_Plugins}
			)

		file(COPY imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		evoengine_copy_resources(DigitalAgricultureApp ${CMAKE_CURRENT_BINARY_DIR})
	endif ()
endif ()