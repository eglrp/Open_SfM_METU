#ifndef OPEN_SFM_METU_INDEXED_FEATURE_MATCH_HPP
#define OPEN_SFM_METU_INDEXED_FEATURE_MATCH_HPP



namespace Open_SfM_METU {
namespace matching {


struct IndexedFeatureMatch {

	IndexedFeatureMatch() {}
	IndexedFeatureMatch(int f1_ind, int f2_ind, float dist) : feature1_ind(f1_ind), feature2_ind(f2_ind), distance(dist) {}

	// Index of the feature in the first image.
	int feature1_ind;
	// Index of the feature in the second image.
	int feature2_ind;
	// Distance between the two features.
	float distance;



};

// Used for sorting a vector of the feature matches.
inline bool CompareFeaturesByDistance(const IndexedFeatureMatch& feature1,
                                      const IndexedFeatureMatch& feature2) {
  return feature1.distance < feature2.distance;
}


}
}


#endif // OPEN_SFM_METU_INDEXED_FEATURE_MATCH_HPP