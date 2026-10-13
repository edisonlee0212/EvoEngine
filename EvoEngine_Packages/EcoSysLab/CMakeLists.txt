set(ECOSYSLAB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(ECOSYSLAB_INCLUDES
	${ECOSYSLAB_DIRECTORY}
	${ECOSYSLAB_DIRECTORY}/include
	${ECOSYSLAB_DIRECTORY}/include/MeshFormation
	${ECOSYSLAB_DIRECTORY}/include/Physics/DynamicSkeleton
	${ECOSYSLAB_DIRECTORY}/include/Physics/DynamicStrands
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/Descriptors
	${ECOSYSLAB_DIRECTORY}/include/Utilities/SpatialCompetition
	${ECOSYSLAB_DIRECTORY}/include/Physics
	${ECOSYSLAB_DIRECTORY}/include/Utilities/TreeIO
	${ECOSYSLAB_DIRECTORY}/include/Volumes
	${ECOSYSLAB_DIRECTORY}/include/ClimateModel
	${ECOSYSLAB_DIRECTORY}/include/SoilModel
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/StrandModel
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/ShootModel
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/RootModel
	${ECOSYSLAB_DIRECTORY}/include/TreeModel/DynamicTreeStrandGraph
	${ECOSYSLAB_DIRECTORY}/include/TreeModel
	${ECOSYSLAB_DIRECTORY}/include/Utilities
	${ECOSYSLAB_DIRECTORY}/include/Structures
	${ECOSYSLAB_DIRECTORY}/include/Reconstruction
	${ECOSYSLAB_DIRECTORY}/include/VoronoiMeshes
	)
file(GLOB_RECURSE ECOSYSLAB_HEADERS ${ECOSYSLAB_DIRECTORY}/include/*.hpp)
file(GLOB_RECURSE ECOSYSLAB_SOURCES ${ECOSYSLAB_DIRECTORY}/src/*.cpp)

add_library(EcoSysLabPackage
	SHARED
	${ECOSYSLAB_HEADERS}
	${ECOSYSLAB_SOURCES}
	)
target_compile_definitions(EcoSysLabPackage
	PUBLIC
	ECOSYSLAB_PACKAGE
	BILLBOARD_CLOUDS_PACKAGE
	PRIVATE
	${EVOENGINE_SDK_DEFS}
	${EVOENGINE_SERVICES_DEFS}
	)
target_include_directories(EcoSysLabPackage
	PUBLIC
	${EVOENGINE_SDK_INCLUDES}
	${ECOSYSLAB_INCLUDES}
	PRIVATE
	${EVOENGINE_SERVICES_INCLUDES}
	)
target_link_libraries(EcoSysLabPackage
	PUBLIC
	BillboardCloudsPackage
	PRIVATE
	EvoEngine_SDK
	${EvoEngine_Services}
	)

set(ECOSYSLAB_PCH ${ECOSYSLAB_DIRECTORY}/include/EcoSysLab_PCH.hpp)
evoengine_configure_runtime_package(EcoSysLabPackage
	EXTRA_PCH ${ECOSYSLAB_PCH}
	RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Internals
	)

find_package(CGAL)
if(CGAL_FOUND)
	target_link_libraries(EcoSysLabPackage PRIVATE CGAL::CGAL)
	target_compile_definitions(EcoSysLabPackage PRIVATE USE_CGAL)
	message(STATUS "EcoSysLab Package: CGAL found and linked.")
	if(WIN32)
		# CGAL links GMP/MPFR shared libs. Package shadow-copy loads only EcoSysLabPackage.dll,
		# so these must sit next to the host executable (Windows DLL search) and in runtime templates.
		set(_evo_cgal_triplet "x64-windows")
		if(DEFINED VCPKG_TARGET_TRIPLET AND NOT "${VCPKG_TARGET_TRIPLET}" STREQUAL "")
			set(_evo_cgal_triplet "${VCPKG_TARGET_TRIPLET}")
		endif()
		set(_evo_gmp_release "")
		set(_evo_mpfr_release "")
		set(_evo_gmp_debug "")
		set(_evo_mpfr_debug "")
		if(DEFINED VCPKG_INSTALLED_DIR)
			set(_evo_gmp_release "${VCPKG_INSTALLED_DIR}/${_evo_cgal_triplet}/bin/gmp-10.dll")
			set(_evo_mpfr_release "${VCPKG_INSTALLED_DIR}/${_evo_cgal_triplet}/bin/mpfr-6.dll")
			set(_evo_gmp_debug "${VCPKG_INSTALLED_DIR}/${_evo_cgal_triplet}/debug/bin/gmp-10.dll")
			set(_evo_mpfr_debug "${VCPKG_INSTALLED_DIR}/${_evo_cgal_triplet}/debug/bin/mpfr-6.dll")
		endif()
		if(NOT EXISTS "${_evo_gmp_release}")
			set(_evo_gmp_release "${_evo_gmp_debug}")
		endif()
		if(NOT EXISTS "${_evo_mpfr_release}")
			set(_evo_mpfr_release "${_evo_mpfr_debug}")
		endif()
		if(EXISTS "${_evo_gmp_release}" AND EXISTS "${_evo_mpfr_release}")
			if(NOT EXISTS "${_evo_gmp_debug}")
				set(_evo_gmp_debug "${_evo_gmp_release}")
			endif()
			if(NOT EXISTS "${_evo_mpfr_debug}")
				set(_evo_mpfr_debug "${_evo_mpfr_release}")
			endif()
			set(_evo_gmp_dll "$<IF:$<CONFIG:Debug>,${_evo_gmp_debug},${_evo_gmp_release}>")
			set(_evo_mpfr_dll "$<IF:$<CONFIG:Debug>,${_evo_mpfr_debug},${_evo_mpfr_release}>")
			set_property(GLOBAL APPEND PROPERTY EVOENGINE_EXTRA_RUNTIME_PAYLOAD_LIBRARIES
				"${_evo_gmp_dll}" "${_evo_mpfr_dll}")
			# Beside the host exe (required for shadow-loaded packages) and beside the package DLL.
			add_custom_command(TARGET EcoSysLabPackage POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${_evo_gmp_dll}" "$<TARGET_FILE_DIR:EcoSysLabPackage>/../gmp-10.dll"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${_evo_mpfr_dll}" "$<TARGET_FILE_DIR:EcoSysLabPackage>/../mpfr-6.dll"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${_evo_gmp_dll}" "$<TARGET_FILE_DIR:EcoSysLabPackage>/gmp-10.dll"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${_evo_mpfr_dll}" "$<TARGET_FILE_DIR:EcoSysLabPackage>/mpfr-6.dll"
				VERBATIM)
			if(DEFINED EVOENGINE_APP_INSTALL_RUNTIME_DIR)
				evoengine_deploy_runtime_file(EcoSysLabPackage "${_evo_gmp_dll}" "${EVOENGINE_APP_INSTALL_RUNTIME_DIR}")
				evoengine_deploy_runtime_file(EcoSysLabPackage "${_evo_mpfr_dll}" "${EVOENGINE_APP_INSTALL_RUNTIME_DIR}")
			endif()
			install(FILES "${_evo_gmp_release}" "${_evo_mpfr_release}"
				DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}"
				CONFIGURATIONS Release RelWithDebInfo MinSizeRel
				OPTIONAL)
			install(FILES "${_evo_gmp_debug}" "${_evo_mpfr_debug}"
				DESTINATION "${EVOENGINE_INSTALL_APP_RUNTIME_DIR}"
				CONFIGURATIONS Debug
				OPTIONAL)
		else()
			message(WARNING "EcoSysLab Package: CGAL linked but gmp-10.dll/mpfr-6.dll were not found beside vcpkg.")
		endif()
	endif()
else()
	message(WARNING "EcoSysLab Package: CGAL not found. Some features may be disabled.")
endif()

if(EVOENGINE_WITH_EDITOR)
  add_subdirectory(Editor)
endif()

target_link_libraries(EcoSysLabPackage PRIVATE kinDS)
