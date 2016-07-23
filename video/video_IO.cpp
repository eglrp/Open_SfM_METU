

#include "Open_SfM_METU/video/video_IO.hpp"
#include "Open_SfM_METU/video/ffmpeg_sampler.hpp"


#include <cstring>
#include <iostream>
#include <cmath>


using namespace std;

namespace Open_SfM_METU {
namespace video {
    
int readVideo(const char *  inPath, const char *  outPath){

    std::cout << "inside read video IO " << std::endl;

    FfmpegSampler ffmpegsampler;

    ffmpegsampler.SampleVideo(inPath, outPath);

    return 0;
}



}
}



