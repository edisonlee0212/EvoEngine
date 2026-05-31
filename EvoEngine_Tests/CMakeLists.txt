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

add_executable(EvoEngine_RenderTests
	Rendering/RenderDemoPythonApiTest.cpp
	)
target_link_libraries(EvoEngine_RenderTests
	PRIVATE
	GTest::gtest_main
	)
target_include_directories(EvoEngine_RenderTests
	PRIVATE
	${3RDPARTY_DIR}/stb/stb
	)
target_compile_definitions(EvoEngine_RenderTests
	PRIVATE
	EVOENGINE_TEST_SOURCE_DIR="${EVOENGINE_TEST_SOURCE_DIR_NORMALIZED}"
	EVOENGINE_TEST_PYTHON_EXECUTABLE="${EVOENGINE_TEST_PYTHON_EXECUTABLE_NORMALIZED}"
	EVOENGINE_TEST_SCRIPT_DIR="${EVOENGINE_TEST_SCRIPT_DIR_NORMALIZED}"
	EVOENGINE_PYEVOENGINE_DIR="$<TARGET_FILE_DIR:PyEvoEngine>"
	EVOENGINE_RENDER_TEST_DIR="$<TARGET_FILE_DIR:EvoEngine_RenderTests>"
	)
add_dependencies(EvoEngine_RenderTests PyEvoEngine)
set_property(TARGET EvoEngine_RenderTests PROPERTY FOLDER Tests)

gtest_discover_tests(EvoEngine_RenderTests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_RenderTests>"
	PROPERTIES LABELS "render,gpu,python"
	)
