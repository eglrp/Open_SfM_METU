#ifndef OPEN_SFM_METU_FEATURE_MATCHER_HPP
#define OPEN_SFM_METU_FEATURE_MATCHER_HPP

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>  // NOLINT
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>


#include <glog/logging.h>
#include <Eigen/Core>

#include "Open_SfM_METU/feature/keypoint.hpp"
#include "Open_SfM_METU/matching/alignment.h"
#include "Open_SfM_METU/matching/feature_matcher_options.hpp"
#include "Open_SfM_METU/matching/feature_correspondence.hpp"
#include "Open_SfM_METU/matching/image_pair_match.hpp"


namespace Open_SfM_METU {
namespace matching {

// This struct is used by the internal cache to hold keypoints and descriptors
// when the are retrieved from the cache.
struct KeypointsAndDescriptors {
  std::string image_name;
  std::vector<feature::Keypoint> keypoints;
  std::vector<Eigen::VectorXf> descriptors;
};

// Class for matching features between images. The intended use for these
// classes is for matching photos in image collections, so all pairwise matches
// are computed. Matching with geometric verification is also possible. Typical
// use case is:
//   FeatureMatcherOptions matcher_options;
//   FeatureMatcher matcher(matcher_options);
//   for (int i = 0; i < num_images_to_match; i++) {
//     matcher.AddImage(image_names[i], keypoints[i], descriptors[i]);
//     // Or, you could add the image with known intrinsics for use during
//     // geometric verification.
//     matcher.AddImage(image_names[i], keypoints[i],
//                      descriptors[i], intrinsics[i]);
//   }
//   std::vector<ImagePairMatch> matches;
//   matcher.MatchImages(&matches);
//   // Or, with geometric verification:
//   VerifyTwoViewMatchesOptions geometric_verification_options;
//   matcher.MatchImages(geometric_verification_options, &matches);
//
// The matches and match quality depend on the options passed to the feature
// matching.
template <class DistanceMetric> class FeatureMatcher {

	public:
		typedef typename DistanceMetric::DistanceType DistanceType;

		explicit FeatureMatcher(const FeatureMatcherOptions& matcher_options);
		virtual ~FeatureMatcher() {}	

		// Adds an image to the matcher with no known intrinsics for this image. The
		// caller still owns the keypoints and descriptors so they must remain valid
		// objects throughout the matching. The image name must be a unique identifier
		// for the image.
		
		virtual void AddImage(const std::string& image_name, const std::vector<feature::Keypoint>& keypoints, const std::vector<Eigen::VectorXf>& descriptors);		

		// Matches features between all images. No geometric verification is
		// performed. Only the matches which pass the have greater than
		// min_num_feature_matches are returned.
		
		// virtual void MatchImages(std::vector<ImagePairMatch>* matches);

		// Set the image pairs that will be matched when MatchImages or
		// MatchImagesWithGeometricVerification is called. This is an optional method;
		// if it is not called, then all possible image-to-image pairs will be
		// matched. The vector should contain unique pairs of image names that should
		// be matched.
		virtual void SetImagePairsToMatch( const std::vector<std::pair<std::string, std::string> >& pairs_to_match);

	protected:
		// NOTE: This method should be overridden in the subclass implementations!
		// Returns true if the image pair is a valid match.
		// This is pure virtual function
		// Then feature_matcher class is abstract base class and we can not create object of it. 
		virtual bool MatchImagePair(  const KeypointsAndDescriptors& features1,
								      const KeypointsAndDescriptors& features2,
								      std::vector<FeatureCorrespondence>* matched_features) = 0;

		FeatureMatcherOptions matcher_options_;

		// A container for the image names.
		std::vector<std::string> image_names_;

		std::vector<std::pair<std::string, std::string> > pairs_to_match_;




};

// ---------------------- Implementation ------------------------ //



template <class DistanceMetric>
FeatureMatcher<DistanceMetric>::FeatureMatcher(
    const FeatureMatcherOptions& options)
    : matcher_options_(options){
	// Initialize the LRU cache. NOTE: even though the Fetch method will be set up
	// to retreive files from disk, it will only do so if
	// matcher_options_.match_out_of_core is set to true.

	matcher_options_.cache_capacity = std::numeric_limits<int>::max();


}


template <class DistanceMetric>
void FeatureMatcher<DistanceMetric>::SetImagePairsToMatch(
    const std::vector<std::pair<std::string, std::string> >& pairs_to_match) {
  pairs_to_match_ = pairs_to_match;
}


template <class DistanceMetric>
void FeatureMatcher<DistanceMetric>::AddImage(
    const std::string& image_name,
    const std::vector<feature::Keypoint>& keypoints,
    const std::vector<Eigen::VectorXf>& descriptors) {
  image_names_.push_back(image_name);

  
  // Insert the features into the cache.
  std::shared_ptr<KeypointsAndDescriptors> keypoints_and_descriptors(new KeypointsAndDescriptors);
  keypoints_and_descriptors->image_name = image_name;
  keypoints_and_descriptors->keypoints = keypoints;
  keypoints_and_descriptors->descriptors = descriptors;
  
}



}
}

#endif // OPEN_SFM_METU_FEATURE_MATCHER_HPP
