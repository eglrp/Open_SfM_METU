#ifndef OPEN_SFM_METU_VIDEO_FFMPEG_PARAMETERS_HPP_
#define OPEN_SFM_METU_VIDEO_FFMPEG_PARAMETERS_HPP_


// Sift blob feature detector parameters. Since the Sift implementation is based
// on the VLFeat one, please visit (http://www.vlfeat.org/api/sift.html) for
// getting more info about the parameters.
struct FfmpegParameters{
  FfmpegParameters() {}
  FfmpegParameters(int num_octaves,
                 int num_levels,
                 int first_octave) : num_octaves(num_octaves), num_levels(num_levels),first_octave(first_octave) {}

  ~FfmpegParameters() {}

  // Parameters.
  // Blob feature detector params.
  int num_octaves = -1;
  int num_levels = 3;
  int first_octave = 0;
};


#endif  // OPEN_SFM_METU_VIDEO_FFMPEG_PARAMETERS_HPP_
