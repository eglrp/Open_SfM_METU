add_subdirectory (calibration)
add_subdirectory (image)
add_subdirectory (feature)
add_subdirectory (nonfree)
add_subdirectory (matching)
add_subdirectory (solvers)
add_subdirectory (math)
add_subdirectory (video)


#INSTALL RULES
INSTALL(
  DIRECTORY .
  DESTINATION include/Open_SfM_METU
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)