# Copy entries instead of the source root so the destination keeps its permissions.
file(GLOB copy_sources "${SOURCE}/*")
file(COPY ${copy_sources} DESTINATION "${DESTINATION}")
