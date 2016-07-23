#ifndef VIDEO_SAMPLER_HPP
#define VIDEO_SAMPLER_HPP


#include <vector>

#include <string>
#include <iostream>

#include <Eigen/Dense>
#include <stdio.h>


namespace Open_SfM_METU {
namespace video {


	// A pure virtual class for video sampling. 
	// We assume that VideoSampling only use 
	class VideoSampler {

		public:

			VideoSampler() {}
			virtual ~VideoSampler() {}

			// Use this method to initizalize eny internals. 
			virtual bool Initialize() { return true; }

			virtual bool SampleVideo(const char * inputPath, const char * outputPath) = 0;

			

	};


}	// namespace video
}   // namespace Open_SfM_METU





#endif