



#include "Open_SfM_METU/feature/descriptor_extractor.hpp"
#include <Eigen/Core>
#include <vector>


#include "Open_SfM_METU/feature/keypoint.hpp"
#include "Open_SfM_METU/image/image.hpp"

namespace Open_SfM_METU {
namespace feature {

	// Compute the descriptor for multiple keypoints in a given image.
	// There is an important notice here. 
	// Input image is grayscale and uchar format. 
	// Before adding any descriptor algorithm we need to check that if it is fit in our function

	bool DescriptorExtractor::ComputeDescriptors(const image::Image<unsigned char>& image, std::vector<Keypoint>* keypoints, std::vector<Eigen::VectorXf>* descriptors){


		descriptors->reserve(keypoints->size());	// Since sparse matirx is used here from Eigen library. 

		auto keypoint_it = keypoints->begin();


		while (keypoint_it != keypoints->end()) {
			Eigen::VectorXf descriptor;
			if (!ComputeDescriptor(image,
			                       *keypoint_it,
			                       &descriptor)) {
			  keypoint_it = keypoints->erase(keypoint_it);
			  continue;
			}


			descriptors->push_back(descriptor);
		}
		return true;

	}


}
}