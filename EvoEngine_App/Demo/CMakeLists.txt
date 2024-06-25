if (${CMAKE_BINARY_DIR} STREQUAL ${PROJECT_BINARY_DIR})
	option(EVOENGINE_BUILD_APP_DEMO "Build Demo application" ON)
	if(EVOENGINE_BUILD_APP_DEMO)
		set(DEFAULT_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Resources/TestingProject/)
		set(DEFAULT_PROJECT_DEF EVOENGINE_DEFAULT_PROJECT_FOLDER=\"${DEFAULT_PROJECT_DIR}\")
	
		file(GLOB PLANET_SOURCES_LOCAL ${CMAKE_CURRENT_SOURCE_DIR}/Planet/src/*.cpp)
		file(GLOB STAR_CLUSTER_SOURCES_LOCAL ${CMAKE_CURRENT_SOURCE_DIR}/StarCluster/src/*.cpp)
		add_library(Universe
			STATIC
			${PLANET_SOURCES_LOCAL}
			${STAR_CLUSTER_SOURCES_LOCAL}
			)
		target_precompile_headers(Universe
			PRIVATE
			${EVOENGINE_PCHS}
			)
		target_include_directories(Universe
			PRIVATE
			${EVOENGINE_INCLUDES}
			${CMAKE_CURRENT_SOURCE_DIR}/Planet/include
			${CMAKE_CURRENT_SOURCE_DIR}/StarCluster/include
			)
		target_compile_definitions(Universe
			PRIVATE
			${DEFAULT_PROJECT_DEF}
			${EVOENGINE_DEFS}
			)

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
			${CMAKE_CURRENT_SOURCE_DIR}/Planet/include
			${CMAKE_CURRENT_SOURCE_DIR}/StarCluster/include
			)
		target_precompile_headers(DemoApp
			PRIVATE
			${EVOENGINE_PCHS}
			)
		target_link_libraries(DemoApp
			EvoEngine_SDK
			${EvoEngine_Plugins}
			Universe
			)
		file(COPY imgui.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		evoengine_copy_resources(DemoApp ${CMAKE_CURRENT_BINARY_DIR})
	endif ()
endif ()