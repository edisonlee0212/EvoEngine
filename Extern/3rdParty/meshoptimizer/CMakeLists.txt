set(TARGET_NAME meshoptimizer)

add_subdirectory(meshoptimizer EXCLUDE_FROM_ALL)

set(3RDPARTY_LIBS ${3RDPARTY_LIBS} 
    meshoptimizer
    PARENT_SCOPE)

set_property(TARGET ${TARGET_NAME} PROPERTY FOLDER 3rdParty)