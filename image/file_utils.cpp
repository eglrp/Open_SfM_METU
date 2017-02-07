#include <vector>

#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <stdio.h>

#include "Open_SfM_METU/image/file_utils.hpp"



namespace Open_SfM_METU {
namespace image {

bool find_file( const char * dir_path,         // in this directory,
                std::string & file_name,       // search for this name,
                const char * path_found,
                const char * listFileName ){	   // placing path here if found

	std::cout << "directory is " << dir_path << std::endl;

	DIR  *dirFile;
	//struct dirent *dir;
	dirFile = opendir(dir_path);
	if ( dirFile ){

		std::ofstream listFile;
		//listFile.open("imageList_video.txt",std::ofstream::out | std::ofstream::trunc);
		listFile.open(listFileName ,std::ofstream::out | std::ofstream::trunc);
		std::cout << "inside find file " << std::endl;
		
		struct dirent* hFile;
		errno = 0;
		while (( hFile = readdir( dirFile )) != NULL ) 
		{
		 if ( !strcmp( hFile->d_name, "."  )) continue;
		 if ( !strcmp( hFile->d_name, ".." )) continue;

		 // in linux hidden files all start with '.'
		 //if ( gIgnoreHidden && ( hFile->d_name[0] == '.' )) continue;

		 // dirFile.name is the name of the file. Do whatever string comparison 
		 // you want here. Something like:
		 if ( strstr( hFile->d_name, ".ppm" )){
		 	std::cout << " found an .ppm file: " << hFile->d_name << std::endl;
			listFile << dir_path << hFile->d_name << "\n";

		 }
		    
		}

		listFile.close(); 
		closedir( dirFile );

	  
	}

	return true; 

}            


}	// namespace image
}   // namespace Open_SfM_METU

