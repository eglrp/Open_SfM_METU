#ifndef OPEN_SFM_METU_FEATURE_DESCRIPTOR_EXTRACTOR_HPP
#define OPEN_SFM_METU_FEATURE_DESCRIPTOR_EXTRACTOR_HPP

#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include "Open_SfM_METU/feature/keypoint.hpp"
#include "Open_SfM_METU/image/image.hpp"

namespace Open_SfM_METU {
namespace feature {

	typedef image::Image<unsigned char> UcharImage;

	class DescriptorExtractor{

		public:

			DescriptorExtractor();
			virtual ~DescriptorExtractor() {}

			// This method should be called before using any of the descriptor
			// extractors. The constuctor should only be used for get/set methods. Any
			// computationally expensive or non-trivial operations should go in the
			// Initialize method.
			virtual bool Initialize() { return true; }

			// const UcharImage& image
			// Computes a floatdescriptor at a single keypoint.
 			virtual bool ComputeDescriptor(const UcharImage& image, const Keypoint& keypoint, Eigen::VectorXf* descriptor) = 0;

 			// Compute the descriptors for multiple keypoints in a given image. This
			// method will return all descriptors that could be extracted. If any
			// descriptors could not be extracted at a given keypoint, that keypoint will
			// be removed from the container. Returns true on success and false on
			// failure.
			virtual bool ComputeDescriptors(const UcharImage& image, std::vector<Keypoint>* keypoints, std::vector<Eigen::VectorXf>* descriptors);
			

		private:
  			//DISALLOW_COPY_AND_ASSIGN(DescriptorExtractor);

	};

}
}



#endif // OPEN_SFM_METU_FEATURE_DESCRIPTOR_EXTRACTOR_HPP