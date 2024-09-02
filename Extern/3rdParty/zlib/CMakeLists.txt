set(zlib_directory ${CMAKE_CURRENT_SOURCE_DIR}/zlib)

# If set this to ON, zlib will add a 1.dll suffix to the output dll. To avoid a conflict with Chromium's zlib (used
# in some UE's plugins), we have to keep this suffix.
set(BUILD_SHARED_LIBS ON)

set(SKIP_INSTALL_ALL ON CACHE BOOL "" FORCE)
set(ZLIB_BUILD_EXAMPLES OFF CACHE INTERNAL "" FORCE)

add_subdirectory(zlib EXCLUDE_FROM_ALL)

# Redirect the find_packge(zlib) to this directory
set(CMAKE_DISABLE_FIND_PACKAGE_ZLIB FALSE CACHE BOOL "" FORCE)
set(ZLIB_INCLUDE_DIR
  ${zlib_directory}
  ${PROJECT_BINARY_DIR}/Extern/3rdParty/zlib/zlib
  CACHE INTERNAL "" FORCE
)
set(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR} CACHE INTERNAL "" FORCE)
set(ZLIB_LIBRARY zlib CACHE INTERNAL "" FORCE)
set(ZLIB_LIBRARIES ${ZLIB_LIBRARY} CACHE INTERNAL "" FORCE)
set(ZLIB_FOUND TRUE CACHE INTERNAL "" FORCE)

add_library(ZLIB::ZLIB ALIAS zlib)

set_target_properties(zlib PROPERTIES FOLDER "3rdparty")

set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${ZLIB_INCLUDE_DIR} PARENT_SCOPE)

set(3RDPARTY_DLLS ${3RDPARTY_DLLS} 
    ${CMAKE_CURRENT_BINARY_DIR}/zlib/zlib1.dll
    PARENT_SCOPE)

set(3RDPARTY_PDBS ${3RDPARTY_PDBS} 
    ${CMAKE_CURRENT_BINARY_DIR}/zlib/zlib.pdb
    PARENT_SCOPE)