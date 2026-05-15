set(LOG_SCANNING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT TARGET EcoSysLabPackage)
	message("LogScanningPackage requires EcoSysLabPackage and will not be built.")
	return()
endif()

set(LOG_SCANNING_INCLUDES
	${LOG_SCANNING_DIRECTORY}/include
	)
file(GLOB_RECURSE LOG_SCANNING_HEADERS ${LOG_SCANNING_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE LOG_SCANNING_SOURCES ${LOG_SCANNING_DIRECTORY}/src/*.cpp)

add_library(LogScanningPackage
	SHARED
	${LOG_SCANNING_HEADERS}
	${LOG_SCANNING_SOURCES}
	)
target_compile_definitions(LogScanningPackage
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	)	
target_include_directories(LogScanningPackage
	PRIVATE
	${EVOENGINE_INCLUDES}
	${LOG_SCANNING_INCLUDES}
	)

if (WIN32)
	set(PINCHOT_LIB ${3RDPARTY_DIR}/pinchot/pinchot/lib/windows/x64/pinchot.lib)
	# Compiler settings for Windows platform
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif ()

target_link_libraries(LogScanningPackage
	PRIVATE
	EvoEngine_SDK
	EcoSysLabPackage
	${PINCHOT_LIB})

evoengine_configure_runtime_package(LogScanningPackage)

if (WIN32)
	set(PINCHOT_RUNTIME_LIBRARY ${3RDPARTY_DIR}/pinchot/pinchot/lib/windows/x64/pinchot.dll)
	add_custom_command(TARGET LogScanningPackage POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		${PINCHOT_RUNTIME_LIBRARY} $<TARGET_FILE_DIR:LogScanningPackage>
		VERBATIM)
	install(FILES ${PINCHOT_RUNTIME_LIBRARY}
		DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}/Packages"
		OPTIONAL)
endif ()
