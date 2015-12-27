add_subdirectory (calibration)
add_subdirectory (image)

#INSTALL RULES
INSTALL(
  DIRECTORY .
  DESTINATION include/Open_SfM_METU
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)