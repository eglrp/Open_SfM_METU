ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

file(
	GLOB_RECURSE
	image_files_header
	*.hpp
)
file(
	GLOB_RECURSE
	image_files_cpp
	*.cpp
)

# Create a library called “Open_SfM_METU_image” which includes the source file “image.cxx".
# The extension is already found. Any number of sources could be listed here.
#add_library (Open_DL_METU_image image.cpp image.hpp)
ADD_LIBRARY(Open_SfM_METU_video ${image_files_header} ${image_files_cpp})


SET_PROPERTY(TARGET Open_SfM_METU_video PROPERTY FOLDER Open_SfM_METU/Open_SfM_METU)
INSTALL(TARGETS Open_SfM_METU_video DESTINATION lib EXPORT Open_SfM_METU-targets)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/local/include/eigen3)

message(“FFMPEG CHECK at video CMakeList“)


find_package(FFmpeg REQUIRED)

if (FFMPEG_FOUND)
  message("Found FFMPEG/LibAV libraries")
  MESSAGE(${FFMPEG_INCLUDE_DIR})
  #include_directories(${FFMPEG_INCLUDE_DIR})
  #target_link_libraries (ffmpeg-example-bin ${FFMPEG_LIBRARIES})
else (FFMPEG_FOUND)
  message("Can't find libavcodec, libavformat or libavutil. Add them!")
endif (FFMPEG_FOUND)


MESSAGE(“video folder \n”)
MESSAGE(${Open_SfM_METU_INCLUDE_DIRS})


include_directories(SYSTEM ${Open_SfM_METU_INCLUDE_DIRS})

FIND_PACKAGE(OpenMVG REQUIRED)
INCLUDE_DIRECTORIES(${OPENMVG_INCLUDE_DIRS})

# Make sure the compiler can find include files for our calibration library
# when other libraries or executables link to calibration
#target_include_directories (Open_DL_METU_calibration PUBLIC ${Open_DL_METU_INCLUDE_DIRS})
#include_directories (${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Open_SfM_METU_video ${OpenCV_LIBS} ${OPENMVG_LIBRARIES} ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${TIFF_LIBRARIES} ${FFMPEG_LIBRARIES})



