#ifndef OPEN_SFM_METU_FEATURE_MATCHER_UTILS_HPP
#define OPEN_SFM_METU_FEATURE_MATCHER_UTILS_HPP

#include <vector>
#include "Open_SfM_METU/matching/indexed_feature_match.hpp"

namespace Open_SfM_METU {
namespace matching {


// Modifies forward matches so that it removes all matches that are not
// contained in the backwards matches.
void IntersectMatches(const std::vector<IndexedFeatureMatch>& backwards_matches,
                      std::vector<IndexedFeatureMatch>* forward_matches);



}
}



#endif // OPEN_SFM_METU_FEATURE_MATCHER_UTILS_HPP