if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	option(EVOENGINE_BUILD_APP_TEXTURE_BAKING "Build texture baking application" ON)
	if(EVOENGINE_BUILD_APP_TEXTURE_BAKING)
		set(DEFAULT_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Resources/TestingProject/)
		set(DEFAULT_PROJECT_DEF EVOENGINE_DEFAULT_PROJECT_FOLDER=\"${DEFAULT_PROJECT_DIR}\")
	
		add_executable(TextureBakingApp
			TextureBakingApp.cpp
			)
		target_compile_definitions(TextureBakingApp
			PRIVATE
			${DEFAULT_PROJECT_DEF}
			${EVOENGINE_DEFS}
			)
		target_include_directories(TextureBakingApp
			PRIVATE
			${EVOENGINE_INCLUDES}
			)
		target_precompile_headers(TextureBakingApp
			PRIVATE
			${EVOENGINE_PCHS}
			)
		target_link_libraries(TextureBakingApp
			EvoEngine_SDK
			${EvoEngine_Plugins}
			Universe
			)
		file(COPY imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		evoengine_copy_resources(TextureBakingApp ${CMAKE_CURRENT_BINARY_DIR})
	endif ()
endif ()