#ifndef OPEN_SFM_METU_BRUTE_FORCE_MATCHER_HPP
#define OPEN_SFM_METU_BRUTE_FORCE_MATCHER_HPP





#include <Eigen/Core>
#include <glog/logging.h>


#include <algorithm>
#include <vector>

#include "Open_SfM_METU/feature/keypoint.hpp"
#include "Open_SfM_METU/matching/feature_matcher.hpp"
#include "Open_SfM_METU/matching/feature_correspondence.hpp"
#include "Open_SfM_METU/matching/feature_matcher_utils.hpp"
#include "Open_SfM_METU/matching/indexed_feature_match.hpp"



namespace Open_SfM_METU {
namespace matching {

// Performs features matching between two sets of features using a brute force
// matching method.
template <class DistanceMetric>
class BruteForceFeatureMatcher : public FeatureMatcher<DistanceMetric> {


	public:
		
		typedef typename DistanceMetric::DistanceType DistanceType;

		explicit BruteForceFeatureMatcher(const FeatureMatcherOptions& options) : FeatureMatcher<DistanceMetric>(options) {}
  		~BruteForceFeatureMatcher() {}

  	

  		bool MatchImagePair(	const KeypointsAndDescriptors& features1,
      							const KeypointsAndDescriptors& features2,
      							std::vector<FeatureCorrespondence>* matched_featuers) override;

  		bool sampleMemberFunction();

  	private:

  		void GetFilteredMatches(const Eigen::MatrixXf& match_distances, std::vector<IndexedFeatureMatch>* matches) const;

  		void FilterMatches(const Eigen::MatrixXf& match_distances, std::vector<IndexedFeatureMatch>* matches) const;




};

	template <class DistanceMetric>
	bool BruteForceFeatureMatcher<DistanceMetric>::MatchImagePair(const KeypointsAndDescriptors& features1,
																    const KeypointsAndDescriptors& features2,
																    std::vector<FeatureCorrespondence>* matched_features) {
	
		const std::vector<Eigen::VectorXf>& descriptors1 = features1.descriptors;
		const std::vector<feature::Keypoint>& keypoints1 = features1.keypoints;
		const std::vector<Eigen::VectorXf>& descriptors2 = features2.descriptors;
		const std::vector<feature::Keypoint>& keypoints2 = features2.keypoints;



		const double sq_lowes_ratio = this->matcher_options_.lowes_ratio * this->matcher_options_.lowes_ratio;


		DistanceMetric distance;
  		std::vector<IndexedFeatureMatch> matches;


  		// Compute forward matches.
		std::vector<IndexedFeatureMatch> temp_matches(descriptors2.size());
		for (int i = 0; i < descriptors1.size(); i++) {
			for (int j = 0; j < descriptors2.size(); j++) {
			  temp_matches[j] =
			      IndexedFeatureMatch(i, j, distance(descriptors1[i], descriptors2[j]));
			}

		// Get the lowest distance matches.
		std::partial_sort(temp_matches.begin(),
		                  temp_matches.begin() + 2,
		                  temp_matches.end(),
		                  CompareFeaturesByDistance);

		// Add to the matches vector if lowes ratio test is turned off or it is
		// turned on and passes the test.
		if (!this->matcher_options_.use_lowes_ratio ||
		    temp_matches[0].distance < sq_lowes_ratio * temp_matches[1].distance) {
		  matches.emplace_back(temp_matches[0]);
		}
		}

		if (matches.size() < this->matcher_options_.min_num_feature_matches) {
    		return false;
  		}

  		// Compute the symmetric matches, if applicable.
		if (this->matcher_options_.keep_only_symmetric_matches) {
		std::vector<IndexedFeatureMatch> reverse_matches;
		temp_matches.resize(descriptors1.size());
		// Only compute the distances for the valid matches.
		for (int i = 0; i < descriptors2.size(); i++) {
		  for (int j = 0; j < descriptors1.size(); j++) {
		    temp_matches[j] = IndexedFeatureMatch(
		        i, j, distance(descriptors2[i], descriptors1[j]));
		  }

		  // Get the lowest distance matches.
		  std::partial_sort(temp_matches.begin(),
		                    temp_matches.begin() + 2,
		                    temp_matches.end(),
		                    CompareFeaturesByDistance);

		  // Add to the matches vector if lowes ratio test is turned off or it is
		  // turned on and passes the test.
		  if (!this->matcher_options_.use_lowes_ratio ||
		      temp_matches[0].distance <
		          sq_lowes_ratio * temp_matches[1].distance) {
		    reverse_matches.emplace_back(temp_matches[0]);
		  }
		}
			IntersectMatches(reverse_matches, &matches);
		}

		if (matches.size() < this->matcher_options_.min_num_feature_matches) {
    		return false;
  		}

  		// Convert to FeatureCorrespondences and return true
		matched_features->resize(matches.size());
		for (int i = 0; i < matches.size(); i++) {
			const feature::Keypoint& keypoint1 = keypoints1[matches[i].feature1_ind];
			const feature::Keypoint& keypoint2 = keypoints2[matches[i].feature2_ind];
			matched_features->at(i).feature1 = Feature(keypoint1.x(), keypoint1.y());
			matched_features->at(i).feature2 = Feature(keypoint2.x(), keypoint2.y());
		}

		return true;

	}

}	// namespace matching
}	// namespace Open_SfM_METU


#endif // OPEN_SFM_METU_BRUTE_FORCE_MATCHER_HPP