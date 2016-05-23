#ifndef OPEN_SFM_METU_SOLVERS_ESTIMATE_FOCAL_LENGTH_LAMBDA_PAIRS_HPP
#define OPEN_SFM_METU_SOLVERS_ESTIMATE_FOCAL_LENGTH_LAMBDA_PAIRS_HPP


#include <Eigen/Core>
#include <vector>

#include "Open_SfM_METU/matching/feature_correspondence.hpp"
#include "Open_SfM_METU/solvers/create_and_initialize_ransac_variant.hpp"
#include "Open_SfM_METU/solvers/estimator.hpp"


namespace Open_SfM_METU {
namespace solvers {


// Estimates the focal length and radial distortion coefficient pairs by utilizing 
// "Simultaneous linear estimation of multiple view geometry and lens distortion"



bool EstimateFocalLengthAndRadialDistortionPairs(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    std::vector<matching::FeatureCorrespondence>& unnormalized_correspondences,
    Eigen::Matrix3d* essential_matrix,
    RansacSummary* ransac_summary,
    std::vector<double>* all_l_values,
    std::vector<double>* all_f_values,
    std::vector<int>* FOV_interval);

}
}





#endif // OPEN_SFM_METU_SOLVERS_ESTIMATE_FUNDAMENTAL_MATRIX_HPP