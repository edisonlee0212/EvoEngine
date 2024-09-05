if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	option(EVOENGINE_BUILD_APP_DEMO "Build Demo application" ON)
	if(EVOENGINE_BUILD_APP_DEMO)
		set(DEFAULT_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Resources/TestingProject/)
		set(DEFAULT_PROJECT_DEF EVOENGINE_DEFAULT_PROJECT_FOLDER=\"${DEFAULT_PROJECT_DIR}\")
		add_executable(DemoApp
			Demo.cpp
			)
		target_compile_definitions(DemoApp
			PRIVATE
			${DEFAULT_PROJECT_DEF}
			${EVOENGINE_DEFS}
			)
		target_include_directories(DemoApp
			PRIVATE
			${EVOENGINE_INCLUDES}
			)
		target_precompile_headers(DemoApp
			PRIVATE
			${EVOENGINE_PCHS}
			)
		target_link_libraries(DemoApp
			EvoEngine_SDK
			${EvoEngine_Plugins}
			)
		file(COPY imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		evoengine_copy_resources(DemoApp ${CMAKE_CURRENT_BINARY_DIR})
	endif ()
endif ()