add_subdirectory(VulkanMemoryAllocator EXCLUDE_FROM_ALL)

set(3RDPARTY_INCLUDES ${3RDPARTY_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/VulkanMemoryAllocator/include PARENT_SCOPE)