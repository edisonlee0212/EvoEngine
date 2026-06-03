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

add_executable(EvoEngine_Tests
	Core/TaskRuntimeTest.cpp
	Core/AssetManagerTest.cpp
	Rendering/RenderDemoPythonApiTest.cpp
	)
target_link_libraries(EvoEngine_Tests
	PRIVATE
	EvoEngine_SDK
	GTest::gtest_main
	)
target_include_directories(EvoEngine_Tests
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
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
	TEST_FILTER AssetManager.*
	PROPERTIES LABELS "core,assets,jobs"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER RenderingDemo.*
	PROPERTIES LABELS "render,gpu,python"
	)

if(TARGET DemoApp)
	set(EVOENGINE_DEMOAPP_SMOKE_TEST_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/DemoAppSmokeTest.run.yaml")
	file(WRITE "${EVOENGINE_DEMOAPP_SMOKE_TEST_CONFIG}"
"mode: smoke_test
demo_setup: Rendering
frames_after_play: 100
max_load_frames: 30000
max_play_frames: 1000
exit_on_complete: true
")
	add_dependencies(EvoEngine_RenderTests DemoApp)
	add_test(NAME DemoApp.PlayLoadedScene100Frames
		COMMAND $<TARGET_FILE:DemoApp> --run-config "${EVOENGINE_DEMOAPP_SMOKE_TEST_CONFIG}"
		)
	set_tests_properties(DemoApp.PlayLoadedScene100Frames
		PROPERTIES
		WORKING_DIRECTORY "$<TARGET_FILE_DIR:DemoApp>"
		LABELS "render,gpu,app"
		TIMEOUT 240
		)
endif()
