add_library(PhysXPhysicsEditorService STATIC src/PhysXEditorLayer.cpp)
target_link_libraries(PhysXPhysicsEditorService PUBLIC PhysXPhysicsService EvoEngine_EditorSDK)
target_include_directories(PhysXPhysicsEditorService PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" ${PHYSX_PHYSICS_INCLUDES})
target_precompile_headers(PhysXPhysicsEditorService PRIVATE ${EVOENGINE_EDITOR_PCH} ${PHYSX_PHYSICS_PCH})
