


#include "Open_SfM_METU/matching/feature_matcher_utils.hpp"

#include <glog/logging.h>
#include <unordered_map>
#include <vector>

#include "Open_SfM_METU/matching/indexed_feature_match.hpp"
#include "Open_SfM_METU/matching/map_util.hpp"


namespace Open_SfM_METU {
namespace matching {


	// Modifies forward matches so that it removes all matches that are not
	// contained in the backwards matches.
	void IntersectMatches(const std::vector<IndexedFeatureMatch>& backwards_matches,
	                      std::vector<IndexedFeatureMatch>* forward_matches) {
	  std::unordered_map<int, int> index_map;
	  index_map.reserve(backwards_matches.size());
	  // Add all feature2 -> feature1 matches to the map.
	  for (const IndexedFeatureMatch& feature_match : backwards_matches) {
	    InsertOrDie(&index_map,
	                feature_match.feature1_ind,
	                feature_match.feature2_ind);
	  }

	  // Search the map for feature1 -> feature2 matches that are also present in
	  // the feature2 -> feature1 matches.
	  auto match_iterator = forward_matches->begin();
	  while (match_iterator != forward_matches->end()) {
	    if (match_iterator->feature1_ind !=
	        FindWithDefault(index_map, match_iterator->feature2_ind, -1)) {
	      match_iterator = forward_matches->erase(match_iterator);
	      continue;
	    }

	    ++match_iterator;
	  }
	}


}
}