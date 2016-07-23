#ifndef FFMPEG_SAMPLER_HPP
#define FFMPEG_SAMPLER_HPP


#include <vector>
#include "Open_SfM_METU/video/video_sampler.hpp"
#include "Open_SfM_METU/video/ffmpeg_parameters.hpp"


extern "C" {
	#include <libavcodec/avcodec.h>    
	#include <libavformat/avformat.h>
	#include <libswscale/swscale.h>
}




namespace Open_SfM_METU {
namespace video {


class FfmpegSampler : public VideoSampler {

	public:

		explicit FfmpegSampler(const FfmpegParameters& ffmpeg_params) : ffmpeg_params_(ffmpeg_params) {}
		FfmpegSampler(int num_octaves, int num_levels, int first_octave) : ffmpeg_params_(num_octaves, num_levels, first_octave) {}
		FfmpegSampler() : FfmpegSampler(-1,3,0){}

		~FfmpegSampler();

		bool SampleVideo(const char * inputPath, const char * outputPath);

	private: 

		const FfmpegParameters ffmpeg_params_;
		char * tempPath;
		
};


}	// namespace video
}   // namespace Open_SfM_METU





#endif