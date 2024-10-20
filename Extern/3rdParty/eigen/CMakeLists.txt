set (TARGET_NAME eigen)

set(EIGEN_BUILD_BTL OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_SPBENCH OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  # Force them on because the auto detection in c-ares may not always find them
  set(standard_math_library_linked_to_automatically 1)
endif()

add_subdirectory(${TARGET_NAME} EXCLUDE_FROM_ALL)
set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/eigen/Eigen PARENT_SCOPE)

set_target_properties(${TARGET_NAME} PROPERTIES FOLDER 3rdParty)