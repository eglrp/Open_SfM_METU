#ifndef FILE_UTILS_HPP
#define FILE_UTILS_HPP


#include <vector>

#include <string>
#include <iostream>

#include <Eigen/Dense>
#include <stdio.h>

#include <dirent.h> 



namespace Open_SfM_METU {
namespace image {

bool find_file( const char * dir_path,         // in this directory,
                std::string & file_name, // search for this name,
                const char * path_found,
                const char * listFileName  );            // placing path here if found


}	// namespace image
}   // namespace Open_SfM_METU





#endif