#ifndef OPEN_SFM_METU_SIFT_DETECTOR_HPP
#define OPEN_SFM_METU_SIFT_DETECTOR_HPP




extern "C" {
	#include <openMVG_dependencies/nonFree/sift/vl/sift.h>
}

#include <vector>
#include "Open_SfM_METU/feature/keypoint.hpp"
#include "Open_SfM_METU/feature/keypoint_detector.hpp"
#include "Open_SfM_METU/feature/sift_parameters.hpp"

#include "Open_SfM_METU/image/image.hpp"



namespace Open_SfM_METU {
namespace feature {


//template<class T> class Image;
typedef image::Image<unsigned char> UcharImage;

class SiftDetector : public KeypointDetector {

	public:
		explicit SiftDetector(const SiftParameters& sift_params) : sift_params_(sift_params), sift_filter_(nullptr) {}
 		SiftDetector(int num_octaves, int num_levels, int first_octave) : sift_params_(num_octaves, num_levels, first_octave), sift_filter_(nullptr) {}
  		SiftDetector() : SiftDetector(-1, 3, 0) {}
  		~SiftDetector();

  	// Given an image, detect keypoints using the sift descriptor.
  	bool DetectKeypoints(const UcharImage& image, std::vector<Keypoint>* keypoints);


	private:
  		const SiftParameters sift_params_;
  		VlSiftFilt* sift_filter_;

  		//DISALLOW_COPY_AND_ASSIGN(SiftDetector);

	

};

}
}



#endif // OPEN_SFM_METU_SIFT_DETECTOR_HPP