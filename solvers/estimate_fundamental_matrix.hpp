#ifndef OPEN_SFM_METU_SOLVERS_ESTIMATE_FUNDAMENTAL_MATRIX_HPP
#define OPEN_SFM_METU_SOLVERS_ESTIMATE_FUNDAMENTAL_MATRIX_HPP


#include <Eigen/Core>
#include <vector>

#include "Open_SfM_METU/matching/feature_correspondence.hpp"
#include "Open_SfM_METU/solvers/create_and_initialize_ransac_variant.hpp"
#include "Open_SfM_METU/solvers/estimator.hpp"



namespace Open_SfM_METU {
namespace solvers {


// Estimates the fundamental matrix from feature correspondences using the 8-pt
// algorithm. The feature correspondences must be normalized such that the
// principal point and focal length has been removed. Returns true if a pose
// could be successfully estimated and false otherwise.


bool EstimateFundamentalMatrix(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<matching::FeatureCorrespondence>& normalized_correspondences,
    Eigen::Matrix3d* essential_matrix,
    RansacSummary* ransac_summary);


}
}





#endif // OPEN_SFM_METU_SOLVERS_ESTIMATE_FUNDAMENTAL_MATRIX_HPP