#ifndef OPEN_SFM_METU_FEATURE_CORRESPONDENCE_HPP
#define OPEN_SFM_METU_FEATURE_CORRESPONDENCE_HPP

#include <string>
#include <vector>

#include "Open_SfM_METU/matching/alignment.h"
#include "Open_SfM_METU/matching/feature_match.hpp"
#include <Eigen/Core>


namespace Open_SfM_METU {
namespace matching {


	// The feature location of two correspondences. These can be pixel coordinates
	// or normalized coordinates.
	struct FeatureCorrespondence {

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			Feature feature1;
			Feature feature2;
			// Here there is a definition for equality in terms of feature correspondence. 
			bool operator==(const FeatureCorrespondence& other) const {
				return (feature1 == other.feature1 && feature2 == other.feature1);
			}

	};

}
}





#endif // OPEN_SFM_METU_FEATURE_CORRESPONDENCE_HPP