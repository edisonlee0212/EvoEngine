set(TARGET_NAME meshoptimizer)

add_subdirectory(meshoptimizer EXCLUDE_FROM_ALL)

set_property(TARGET ${TARGET_NAME} PROPERTY FOLDER "3rdparty")