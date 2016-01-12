ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

file(
	GLOB_RECURSE
	matching_files_header
	*.hpp
)
file(
	GLOB_RECURSE
	matching_files_cpp
	*.cpp
)
# Create a library called “Open_SfM_METU_matching” which includes the source file “matching.cxx".
# The extension is already found. Any number of sources could be listed here.
#add_library (Open_SfM_METU_feature feature.cpp feature.hpp)
ADD_LIBRARY(Open_SfM_METU_matching ${matching_files_header} ${matching_files_cpp})


SET_PROPERTY(TARGET Open_SfM_METU_matching PROPERTY FOLDER Open_SfM_METU/Open_SfM_METU)
INSTALL(TARGETS Open_SfM_METU_matching DESTINATION lib EXPORT Open_SfM_METU-targets)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/local/include/eigen3)

MESSAGE(“feature-matching folder \n”)
MESSAGE(${Open_SfM_METU_INCLUDE_DIRS})

include_directories(SYSTEM ${Open_SfM_METU_INCLUDE_DIRS})




# Make sure the compiler can find include files for our calibration library
# when other libraries or executables link to calibration
#target_include_directories (Open_SfM_METU_feature PUBLIC ${Open_SfM_METU_INCLUDE_DIRS})
#include_directories (${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Open_SfM_METU_matching ${OpenCV_LIBS})



