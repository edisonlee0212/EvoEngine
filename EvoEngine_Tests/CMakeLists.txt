include(FetchContent)
include(GoogleTest)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(
	googletest
	GIT_REPOSITORY https://github.com/google/googletest.git
	GIT_TAG v1.17.0
)
FetchContent_MakeAvailable(googletest)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

file(TO_CMAKE_PATH "${CMAKE_SOURCE_DIR}" EVOENGINE_TEST_SOURCE_DIR_NORMALIZED)
file(TO_CMAKE_PATH "${Python3_EXECUTABLE}" EVOENGINE_TEST_PYTHON_EXECUTABLE_NORMALIZED)
file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Rendering" EVOENGINE_TEST_SCRIPT_DIR_NORMALIZED)

set(EVOENGINE_TEST_SOURCES
	Core/TaskRuntimeTest.cpp
	Core/ProfilerTest.cpp
	Core/AssetManagerTest.cpp
	Core/DemoSceneTest.cpp
	Core/FileManagerTest.cpp
	Core/PathUtilsTest.cpp
	Core/LauncherUtilsTest.cpp
	Core/GpuServiceTest.cpp
	Core/PlatformQueueFamilyTest.cpp
	Core/RenderGraphTest.cpp
	Core/DdgiVolumeTest.cpp
	Core/VolumetricCloudSettingsTest.cpp
	Core/VolumetricCloudShaderTest.cpp
	Core/InspectorRegistryTest.cpp
	Core/SerializationRegistryTest.cpp
	Rendering/RenderDemoPythonApiTest.cpp
	${CMAKE_SOURCE_DIR}/EvoEngine_App/src/DemoProfiles.cpp
	${CMAKE_SOURCE_DIR}/EvoEngine_App/src/LauncherUtils.cpp
	)

if(TARGET EcoSysLabPackage)
	list(APPEND EVOENGINE_TEST_SOURCES
		EcoSysLab/EcoSysLabTreeStrandsTest.cpp
		)
endif()

add_executable(EvoEngine_Tests
	${EVOENGINE_TEST_SOURCES}
	)
target_link_libraries(EvoEngine_Tests
	PRIVATE
	EvoEngine_SDK
	EvoEngine_DemoScene
	GTest::gtest_main
	)
if(TARGET EcoSysLabPackage)
	target_link_libraries(EvoEngine_Tests
		PRIVATE
		EcoSysLabPackage
		)
	foreach(package_runtime_target EcoSysLabPackage BillboardCloudsPackage)
		if(TARGET ${package_runtime_target})
			add_custom_command(TARGET EvoEngine_Tests POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"$<TARGET_FILE:${package_runtime_target}>"
					"$<TARGET_FILE_DIR:EvoEngine_Tests>"
				VERBATIM)
		endif()
	endforeach()
	add_custom_command(TARGET EvoEngine_Tests POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
			"${CMAKE_SOURCE_DIR}/EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources"
			"$<TARGET_FILE_DIR:EvoEngine_Tests>/EcoSysLabResources"
		VERBATIM)
endif()
target_include_directories(EvoEngine_Tests
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${CMAKE_SOURCE_DIR}/EvoEngine_App/include
	${3RDPARTY_DIR}/stb/stb
	)
target_compile_definitions(EvoEngine_Tests
	PRIVATE
	EVOENGINE_TEST_SOURCE_DIR="${EVOENGINE_TEST_SOURCE_DIR_NORMALIZED}"
	EVOENGINE_TEST_PYTHON_EXECUTABLE="${EVOENGINE_TEST_PYTHON_EXECUTABLE_NORMALIZED}"
	EVOENGINE_TEST_SCRIPT_DIR="${EVOENGINE_TEST_SCRIPT_DIR_NORMALIZED}"
	EVOENGINE_PYEVOENGINE_DIR="$<TARGET_FILE_DIR:PyEvoEngine>"
	EVOENGINE_RENDER_TEST_DIR="$<TARGET_FILE_DIR:EvoEngine_Tests>"
	)
add_dependencies(EvoEngine_Tests PyEvoEngine)
evoengine_copy_resources(EvoEngine_Tests "$<TARGET_FILE_DIR:EvoEngine_Tests>")
set_property(TARGET EvoEngine_Tests PROPERTY FOLDER Tests)

add_custom_target(EvoEngine_RenderTests)
add_dependencies(EvoEngine_RenderTests EvoEngine_Tests)
set_property(TARGET EvoEngine_RenderTests PROPERTY FOLDER Tests)

gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER TaskRuntime.*
	PROPERTIES LABELS "core,jobs"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER Profiler.*
	PROPERTIES LABELS "core,profiler,jobs"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER AssetManager.*
	PROPERTIES LABELS "core,assets,jobs"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER ProjectManager.*
	PROPERTIES LABELS "core,project"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER PathUtils.*
	PROPERTIES LABELS "core,filesystem"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER FileManager.*
	PROPERTIES LABELS "core,filesystem"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER PackageManager.*
	PROPERTIES LABELS "core,project"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER LauncherUtils.*
	PROPERTIES LABELS "core,launcher"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER DemoScene.*
	PROPERTIES LABELS "core,demo"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER GpuService.*
	PROPERTIES LABELS "render,gpu,jobs"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER RenderGraph.*
	PROPERTIES LABELS "core,render"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER DdgiVolume.*
	PROPERTIES LABELS "core,render,ddgi"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER VolumetricCloudSettings.*
	PROPERTIES LABELS "core,render,volumetric-cloud"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER VolumetricCloudShader.*
	PROPERTIES LABELS "core,render,volumetric-cloud"
	)
if(TARGET EcoSysLabPackage)
	gtest_discover_tests(EvoEngine_Tests
		WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
		TEST_FILTER EcoSysLab.*
		PROPERTIES LABELS "render,gpu,ecosyslab"
		)
endif()
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER InspectorRegistry.*
	PROPERTIES LABELS "core,editor,registry"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER SerializationRegistry.*
	PROPERTIES LABELS "core,serialization,registry"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER RenderingDemo.*
	PROPERTIES LABELS "render,gpu,python"
	)

if(WIN32 AND TARGET EvoEngineLauncher AND TARGET EvoEngineEditor)
	add_test(NAME Launcher.ProcessBoundaries
		COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/Launcher/launcher_process_smoke.py"
			--launcher "$<TARGET_FILE:EvoEngineLauncher>"
			--editor "$<TARGET_FILE:EvoEngineEditor>"
			--project "${CMAKE_SOURCE_DIR}/Resources/EvoEngine-DemoProjects/Rendering/Rendering.eveproj"
		)
	set_tests_properties(Launcher.ProcessBoundaries
		PROPERTIES
		WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngineLauncher>"
		LABELS "launcher,app"
		TIMEOUT 90
		)
	add_test(NAME Launcher.WindowsUiSmoke
		COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/Launcher/launcher_ui_smoke.py"
			--launcher "$<TARGET_FILE:EvoEngineLauncher>"
		)
	set_tests_properties(Launcher.WindowsUiSmoke
		PROPERTIES
		WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngineLauncher>"
		LABELS "launcher,ui,app"
		TIMEOUT 90
		)
endif()
