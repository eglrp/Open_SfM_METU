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
#add_library (Open_SfM_METU_image image.cpp image.hpp)
ADD_LIBRARY(Open_SfM_METU_image ${image_files_header} ${image_files_cpp})


SET_PROPERTY(TARGET Open_SfM_METU_image PROPERTY FOLDER Open_SfM_METU/Open_SfM_METU)
INSTALL(TARGETS Open_SfM_METU_image DESTINATION lib EXPORT Open_SfM_METU-targets)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/local/include/eigen3)

MESSAGE(“image folder \n”)
MESSAGE(${Open_SfM_METU_INCLUDE_DIRS})

include_directories(SYSTEM ${Open_SfM_METU_INCLUDE_DIRS})

FIND_PACKAGE(OpenMVG REQUIRED)
INCLUDE_DIRECTORIES(${OPENMVG_INCLUDE_DIRS})

# Make sure the compiler can find include files for our calibration library
# when other libraries or executables link to calibration
#target_include_directories (Open_SfM_METU_calibration PUBLIC ${Open_SfM_METU_INCLUDE_DIRS})
#include_directories (${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Open_SfM_METU_image ${OpenCV_LIBS} ${OPENMVG_LIBRARIES} ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${TIFF_LIBRARIES})



