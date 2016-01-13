#ifndef OPEN_SFM_METU_FEATURE_MATCHER_OPTIONS_HPP
#define OPEN_SFM_METU_FEATURE_MATCHER_OPTIONS_HPP


#include <string>

namespace Open_SfM_METU {
namespace matching {

// Options for matching image collections.
struct FeatureMatcherOptions {
  // Number of threads to use in parallel for matching.
  int num_threads = 1;

  // Matching may be performed in core (i.e. all in memory) or out-of-core. For
  // the latter, features are written and read to/from disk as needed (utilizing
  // an LRU cache). The out-of-core strategy is more scalable since the memory
  // footprint is limited. Set this value to false to perform all-in-memory
  // matching.
  bool match_out_of_core = false;

  // Keypoints and descriptors are stored to disk as they are added to the
  // FeatureMatcher. Features will be stored in this directory, which must be a
  // valid writeable directory.
  std::string keypoints_and_descriptors_output_dir = "";

  // We store the descriptors of up to cache_capacity images in the cache at a
  // given time. The higher the cache capacity, the more memory is required to
  // perform image-to-image matching.
  int cache_capacity = 128;

  // Only symmetric matches are kept.
  bool keep_only_symmetric_matches = true;

  // Only keep the matches that pass the lowes ratio test such that the distance
  // of the best best match is less than lowes_ratio of the distance of the
  // second nearest neighbor match.
  bool use_lowes_ratio = true;
  float lowes_ratio = 0.8;

  // Only images that contain more feature matches than this number will be
  // returned.
  int min_num_feature_matches = 30;
};

}  // namespace Open_SfM_METU
}  // namespace matching


#endif  // OPEN_SFM_METU_FEATURE_MATCHER_OPTIONS_HPP
