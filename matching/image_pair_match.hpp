#ifndef OPEN_SFM_METU_IMAGE_PAIR_MATCH_HPP
#define OPEN_SFM_METU_IMAGE_PAIR_MATCH_HPP

#include <string>
#include <vector>

#include "Open_SfM_METU/matching/alignment.h"
#include "Open_SfM_METU/matching/feature_correspondence.hpp"


namespace Open_SfM_METU {
namespace matching {

	struct ImagePairMatch{

		std::string image1;
		std::string image2;

		// If the matches are verified matches then the two view info contains the
		// relative pose information between the images.
		// TwoViewInfo twoview_info;

		// Feature locations in pixel coordinates. If the match is a verified match
  		// then this only contains inlier correspondences.
  		std::vector<FeatureCorrespondence> correspondences;

	};

	struct ImagePairMatchDeprecated {

		// Indices of the matches image pair with respect to the input vectors.
		int image1_index;
		int image2_index;

		// If the matches are verified matches then the two view info contains the
		// relative pose information between the images.
		//TwoViewInfo twoview_info;

		// Feature locations in pixel coordinates. If the match is a verified match
		// then this only contains inlier correspondences.
		std::vector<FeatureCorrespondence> correspondences;




	};

}
}



#endif // OPEN_SFM_METU_IMAGE_PAIR_MATCH_HPP
