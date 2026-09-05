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
	Core/BistroDemoScriptTest.cpp
	Core/Texture2DTest.cpp
	Core/GltfMaterialLayoutTest.cpp
	Core/GltfMaterialConversionTest.cpp
	Core/GltfSceneFeaturesTest.cpp
	Core/GltfRasterMaterialShaderTest.cpp
	Core/GltfRayTracingMaterialShaderTest.cpp
	Core/ShaderCacheTest.cpp
	Core/ShaderBaselineTest.cpp
	Core/VulkanPipelineCacheTest.cpp
	Core/RayCameraShaderVariantCacheTest.cpp
	Core/CameraRenderTechniqueTest.cpp
	Core/DirectionalShadowCascadeFitTest.cpp
	Core/RayCameraHistoryTest.cpp
	Core/PostProcessingRuntimeTest.cpp
	Core/RayTracingSkinnedInstanceTest.cpp
	Core/PunctualLightImportTest.cpp
	Core/GpuServiceTest.cpp
	Core/SdfgiRuntimeTest.cpp
	Core/PlatformQueueFamilyTest.cpp
	Core/RenderGraphTest.cpp
	Core/StrandsMeshShaderTest.cpp
	Core/DdgiVolumeTest.cpp
	Core/ReflectionProbeTest.cpp
	Core/EnvironmentalLightingContractTest.cpp
	Core/EnvironmentalLightingAssetTest.cpp
	Core/EntitySelectionTest.cpp
	Core/EntityBatchInspectorTest.cpp
	Core/StaticSceneRenderTest.cpp
	Core/EditorConsoleContractTest.cpp
	Core/EditorViewportInputTest.cpp
	Core/RenderingDocumentationTest.cpp
	Core/VolumetricCloudSettingsTest.cpp
	Core/VolumetricCloudShaderTest.cpp
	Core/InspectorRegistryTest.cpp
	Core/SerializationRegistryTest.cpp
	Rendering/RenderDemoPythonApiTest.cpp
	${CMAKE_SOURCE_DIR}/EvoEngine_App/src/DemoProfiles.cpp
	${CMAKE_SOURCE_DIR}/EvoEngine_App/src/LauncherUtils.cpp
	)

add_executable(EvoEngine_Tests
	${EVOENGINE_TEST_SOURCES}
	)
target_link_libraries(EvoEngine_Tests
	PRIVATE
	EvoEngine_SDK
	EvoEngine_DemoScene
	GTest::gtest_main
	)
target_include_directories(EvoEngine_Tests
	PRIVATE
	${EVOENGINE_SDK_INCLUDES}
	${CMAKE_SOURCE_DIR}/EvoEngine_App/include
	${3RDPARTY_DIR}/stb/stb
	)
target_compile_definitions(EvoEngine_Tests
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	EVOENGINE_TEST_SOURCE_DIR="${EVOENGINE_TEST_SOURCE_DIR_NORMALIZED}"
	EVOENGINE_TEST_PYTHON_EXECUTABLE="${EVOENGINE_TEST_PYTHON_EXECUTABLE_NORMALIZED}"
	EVOENGINE_TEST_SCRIPT_DIR="${EVOENGINE_TEST_SCRIPT_DIR_NORMALIZED}"
	EVOENGINE_PYEVOENGINE_DIR="$<TARGET_FILE_DIR:PyEvoEngine>"
	EVOENGINE_RENDER_TEST_DIR="$<TARGET_FILE_DIR:EvoEngine_Tests>"
	)
target_precompile_headers(EvoEngine_Tests PRIVATE ${EVOENGINE_SDK_PCH})
add_dependencies(EvoEngine_Tests PyEvoEngine)
evoengine_copy_resources(EvoEngine_Tests "$<TARGET_FILE_DIR:EvoEngine_Tests>")
set_property(TARGET EvoEngine_Tests PROPERTY FOLDER Tests)

if(TARGET DigitalAgriculturePackage)
	add_executable(DigitalAgriculture_Tests Core/DigitalAgricultureDataTest.cpp)
	target_link_libraries(DigitalAgriculture_Tests PRIVATE DigitalAgriculturePackage EvoEngine_SDK yaml-cpp GTest::gtest_main)
	target_compile_definitions(DigitalAgriculture_Tests PRIVATE ${EVOENGINE_SDK_DEFS})
	target_precompile_headers(DigitalAgriculture_Tests PRIVATE
		${EVOENGINE_SDK_PCH}
		${EVOENGINE_PACKAGE_PCH}
		${DIGITAL_AGRICULTURE_PCH}
		)
	add_custom_command(TARGET DigitalAgriculture_Tests POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			$<TARGET_FILE:EvoEngine_SDK>
			$<TARGET_FILE:BillboardCloudsPackage>
			$<TARGET_FILE:EcoSysLabPackage>
			$<TARGET_FILE:DigitalAgriculturePackage>
			$<TARGET_FILE_DIR:DigitalAgriculture_Tests>
		VERBATIM)
	evoengine_copy_resources(DigitalAgriculture_Tests "$<TARGET_FILE_DIR:DigitalAgriculture_Tests>")
	set_property(TARGET DigitalAgriculture_Tests PROPERTY FOLDER Tests)
	gtest_discover_tests(DigitalAgriculture_Tests
		WORKING_DIRECTORY "$<TARGET_FILE_DIR:DigitalAgriculture_Tests>"
		PROPERTIES LABELS "digital-agriculture,serialization"
		)
endif()

if(TARGET UniversePackage)
	add_executable(UniversePackage_Tests Core/UniverseStarClusterTest.cpp Core/UniverseStarPickingTest.cpp Core/UniverseStarFollowTest.cpp Core/UniverseStarDemoTest.cpp)
	target_link_libraries(UniversePackage_Tests PRIVATE UniversePackage EvoEngine_SDK yaml-cpp GTest::gtest_main)
	target_include_directories(UniversePackage_Tests PRIVATE
		${EVOENGINE_SDK_INCLUDES}
		${CMAKE_SOURCE_DIR}/EvoEngine_Packages/Universe/include
		)
	target_compile_definitions(UniversePackage_Tests PRIVATE ${EVOENGINE_SDK_DEFS})
	target_precompile_headers(UniversePackage_Tests PRIVATE
		${EVOENGINE_SDK_PCH}
		${EVOENGINE_PACKAGE_PCH}
		)
	add_custom_command(TARGET UniversePackage_Tests POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			$<TARGET_FILE:EvoEngine_SDK>
			$<TARGET_FILE:UniversePackage>
			$<TARGET_FILE_DIR:UniversePackage_Tests>
		VERBATIM)
	set_property(TARGET UniversePackage_Tests PROPERTY FOLDER Tests)
	gtest_discover_tests(UniversePackage_Tests)
endif()

if(TARGET EcoSysLabPackage)
	add_executable(EcoSysLab_Tests Core/DynamicStrandsBundleTest.cpp)
	target_link_libraries(EcoSysLab_Tests PRIVATE EcoSysLabPackage EvoEngine_SDK yaml-cpp GTest::gtest_main)
	target_compile_definitions(EcoSysLab_Tests PRIVATE ${EVOENGINE_SDK_DEFS})
	target_precompile_headers(EcoSysLab_Tests PRIVATE ${EVOENGINE_SDK_PCH})
	add_custom_command(TARGET EcoSysLab_Tests POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			$<TARGET_FILE:EvoEngine_SDK>
			$<TARGET_FILE:BillboardCloudsPackage>
			$<TARGET_FILE:EcoSysLabPackage>
			$<TARGET_FILE_DIR:EcoSysLab_Tests>
		VERBATIM)
	evoengine_copy_resources(EcoSysLab_Tests "$<TARGET_FILE_DIR:EcoSysLab_Tests>")
	set_property(TARGET EcoSysLab_Tests PROPERTY FOLDER Tests)
	gtest_discover_tests(EcoSysLab_Tests
		WORKING_DIRECTORY "$<TARGET_FILE_DIR:EcoSysLab_Tests>"
		PROPERTIES LABELS "ecosyslab,dynamic-strands"
		)
	add_test(NAME DynamicStrandsBundle.ShaderInventory
		COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/Scripts/check_dynamic_strands_bundle_shaders.py")
	set_tests_properties(DynamicStrandsBundle.ShaderInventory PROPERTIES LABELS "ecosyslab,dynamic-strands,shaders")
endif()

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
	TEST_FILTER BistroDemoScript.*
	PROPERTIES LABELS "core,demo,script"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER Texture2D.*
	PROPERTIES LABELS "core,texture"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER GltfMaterialLayout.*
	PROPERTIES LABELS "core,render,material"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER GltfMaterialConversion.*
	PROPERTIES LABELS "core,render,material"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER GltfSceneFeatures.*
	PROPERTIES LABELS "core,render,material,shader-variants"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER GltfRasterMaterial.*
	PROPERTIES LABELS "core,render,material"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER GltfRayTracingMaterial.*
	PROPERTIES LABELS "core,render,material"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER ShaderCache.*
	PROPERTIES LABELS "core,render,shader-cache"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER ShaderBaseline.*
	PROPERTIES LABELS "core,render,shader-baseline"
	)
add_test(NAME ShaderPolicy.Sdk
	COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/Scripts/check_sdk_shader_policy.py"
	)
set_tests_properties(ShaderPolicy.Sdk
	PROPERTIES
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
	LABELS "core,render,shader-policy"
	)
add_test(NAME ShaderPolicy.FirstParty
	COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/Scripts/check_first_party_shader_policy.py"
	)
set_tests_properties(ShaderPolicy.FirstParty
	PROPERTIES
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
	LABELS "core,render,shader-policy"
	)
add_test(NAME TextureAccessPolicy.FirstParty
	COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/Scripts/check_texture_access_policy.py"
	)
set_tests_properties(TextureAccessPolicy.FirstParty
	PROPERTIES
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
	LABELS "core,render,shader-policy,texture"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER VulkanPipelineCache.*:RayCameraShaderVariantCache.*
	PROPERTIES LABELS "core,render,pipeline-cache,shader-variants"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER GraphicsInitializationSettings.*:CameraRenderTechnique.*
	PROPERTIES LABELS "core,render,camera"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER DirectionalShadowCascadeFit.*
	PROPERTIES LABELS "core,render,shadow"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER RayCameraHistory.*:PostProcessingRuntime.*
	PROPERTIES LABELS "core,render,camera,post-processing"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER RayTracingSkinned.*
	PROPERTIES LABELS "core,render,ray-tracing"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER PunctualLightImport.*
	PROPERTIES LABELS "core,render,ray-tracing,import"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER DemoScene.*
	PROPERTIES LABELS "core,demo"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER StaticBlasBuilder.*
	PROPERTIES LABELS "core,render,ray-tracing"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER GpuService.*
	PROPERTIES LABELS "render,gpu,jobs"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER SdfgiCapabilities.*:SdfgiRuntime.*:SdfgiResources.*:SdfgiScene.*:SdfgiVoxelization.*:SdfgiPreprocess.*
	PROPERTIES LABELS "render,sdfgi"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER RenderGraph.*
	PROPERTIES LABELS "core,render"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER StrandsMeshShader.*
	PROPERTIES LABELS "core,render,strands"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER DdgiVolume.*
	PROPERTIES LABELS "core,render,ddgi"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER ReflectionProbe.*
	PROPERTIES LABELS "core,render,reflection-probe"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER EnvironmentalLightingContract.*
	PROPERTIES LABELS "core,render,environmental-lighting"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER EntitySelection.*
	PROPERTIES LABELS "core,editor,selection"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER EntityBatchInspector.*
	PROPERTIES LABELS "core,editor,selection"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER StaticSceneRender.*
	PROPERTIES LABELS "core,render,editor"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER EditorConsole.*
	PROPERTIES LABELS "core,editor,console"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER EditorViewportInput.*
	PROPERTIES LABELS "core,editor,input"
	)
gtest_discover_tests(EvoEngine_Tests
	WORKING_DIRECTORY "$<TARGET_FILE_DIR:EvoEngine_Tests>"
	TEST_FILTER RenderingDocumentation.*
	PROPERTIES LABELS "core,render,documentation"
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
		TIMEOUT 240
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
