set(LOG_GRADING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT TARGET EcoSysLabPlugin)
	message("LogGradingPackage requires EcoSysLabPlugin and will not be built.")
	return()
endif()

set(LOG_GRADING_INCLUDES
	${LOG_GRADING_DIRECTORY}/include
	)
file(GLOB_RECURSE LOG_GRADING_HEADERS ${LOG_GRADING_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE LOG_GRADING_SOURCES ${LOG_GRADING_DIRECTORY}/src/*.cpp)

add_library(LogGradingPackage
	SHARED
	${LOG_GRADING_HEADERS}
	${LOG_GRADING_SOURCES}
	)
target_compile_definitions(LogGradingPackage
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	)	
target_include_directories(LogGradingPackage
	PRIVATE
	${EVOENGINE_INCLUDES}
	${LOG_GRADING_INCLUDES}
	)
target_link_libraries(LogGradingPackage
	PRIVATE
	EvoEngine_SDK
	EcoSysLabPlugin
	)
evoengine_configure_runtime_package(LogGradingPackage)
