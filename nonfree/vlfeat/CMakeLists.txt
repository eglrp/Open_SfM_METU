
# libs should be static
SET(BUILD_SHARED_LIBS OFF)

include_directories(./vl)
set(vl_sources
  vl/generic.c
  vl/host.c
  vl/imopv.c
  vl/imopv_sse2.c
  vl/mathop.c
  vl/mathop_sse2.c
  vl/random.c
  vl/sift.c)
set_source_files_properties(${vl_sources} PROPERTIES LANGUAGE C)
add_library(vlfeat ${vl_sources})

INSTALL(TARGETS vlfeat DESTINATION lib EXPORT Open_SfM_METU-targets)
SET_PROPERTY(TARGET vlfeat PROPERTY FOLDER Open_SfM_METU/nonFree)

INSTALL(
  DIRECTORY .
  DESTINATION include/Open_SfM_METU_dependencies/nonFree/vlfeat
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)


