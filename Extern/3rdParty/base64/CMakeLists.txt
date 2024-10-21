set (TARGET_NAME base64)
project (${TARGET_NAME})

set (PROJECT_HEADERS
    base64/include/base64.h
)

set (PROJECT_SOURCES
    base64/include/base64.cpp
)

add_library (
    ${TARGET_NAME}
    STATIC
    ${PROJECT_HEADERS}
    ${PROJECT_SOURCES}
)

target_include_directories (
    ${TARGET_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library (
    ${COMMON_LIB}::${TARGET_NAME}
    ALIAS
    ${TARGET_NAME}
)
set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/base64/include PARENT_SCOPE)
set(3RDPARTY_LIBS ${3RDPARTY_LIBS} 
    ${TARGET_NAME}
    PARENT_SCOPE)
set_target_properties(${TARGET_NAME} PROPERTIES FOLDER 3rdParty)