#ifndef OPEN_SFM_METU_FEATURE_SIFT_DESCRIPTOR_EXTRACTOR_HPP
#define OPEN_SFM_METU_FEATURE_SIFT_DESCRIPTOR_EXTRACTOR_HPP

extern "C" {
	#include <Open_SfM_METU/nonFree/vlfeat/vl/sift.h>
}


#include "Open_SfM_METU/feature/descriptor_extractor.hpp"
#include <Eigen/Core>


#include <vector>
#include "Open_SfM_METU/feature/keypoint.hpp"
#include "Open_SfM_METU/image/image.hpp"
#include "Open_SfM_METU/feature/sift_parameters.hpp"



namespace Open_SfM_METU {
namespace feature {

typedef image::Image<unsigned char> UcharImage;

class SiftDescriptorExtractor : public DescriptorExtractor {

	public:

		//  We only implement the standard 128-dimension descriptor. Specify the
		//  number of image octaves, number of scale levels per octave, and where the
		//  first octave should start.
		explicit SiftDescriptorExtractor(const SiftParameters& detector_params) : sift_params_(detector_params), sift_filter_(nullptr) {}
		SiftDescriptorExtractor(int num_octaves, int num_levels, int first_octave) : sift_params_(num_octaves, num_levels, first_octave, 10.0f, 255.0 * 0.02 / num_levels), sift_filter_(nullptr) {}
		SiftDescriptorExtractor() : SiftDescriptorExtractor(-1, 3, -1) {}

		~SiftDescriptorExtractor();

		// Computes a descriptor at a single keypoint.
	  	bool ComputeDescriptor(const UcharImage& image, const Keypoint& keypoint, Eigen::VectorXf* descriptor);

		// Compute multiple descriptors for keypoints from a single image.
		bool ComputeDescriptors(const UcharImage& image, std::vector<Keypoint>* keypoints, std::vector<Eigen::VectorXf>* descriptors);

		// Detect keypoints using the Sift keypoint detector and extracts them at the
		// same time.
		//sbool DetectAndExtractDescriptors(const FloatImage& image, std::vector<Keypoint>* keypoints, std::vector<Eigen::VectorXf>* descriptors);


	private:
		const SiftParameters sift_params_;
		VlSiftFilt* sift_filter_;

		//DISALLOW_COPY_AND_ASSIGN(SiftDescriptorExtractor);


};

}
}


#endif // OPEN_SFM_METU_FEATURE_SIFT_DESCRIPTOR_EXTRACTOR_HPP