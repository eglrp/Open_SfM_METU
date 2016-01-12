add_subdirectory (calibration)
add_subdirectory (image)
add_subdirectory (feature)
add_subdirectory (nonfree)
add_subdirectory (matching)

#INSTALL RULES
INSTALL(
  DIRECTORY .
  DESTINATION include/Open_SfM_METU
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)