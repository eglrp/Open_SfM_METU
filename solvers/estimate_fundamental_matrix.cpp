

#include "Open_SfM_METU/solvers/estimate_fundamental_matrix.hpp"

#include <Eigen/Core>
#include <vector>

#include "Open_SfM_METU/matching/feature_correspondence.hpp"

#include "Open_SfM_METU/solvers/pose/eight_point_fundamental_matrix.hpp"

#include "Open_SfM_METU/solvers/estimator.hpp"
#include "Open_SfM_METU/solvers/pose_util.hpp"

#include "Open_SfM_METU/solvers/util_util.hpp"



namespace Open_SfM_METU {
namespace solvers {

namespace {


// An estimator for computing the fundamental matrix from 8 feature
// correspondences. The feature correspondences should be in pixel coordinates.

class FundamentalMatrixEstimator
    : public Estimator<matching::FeatureCorrespondence, Eigen::Matrix3d> {


    public:
    	FundamentalMatrixEstimator() {}

    	// 8 correspondences are needed to determine an fundamental matrix.
		double SampleSize() const { return 8; }

		// Estimates candidate fundamental matrices from correspondences.
		bool EstimateModel(const std::vector<matching::FeatureCorrespondence>& correspondences,
		                 std::vector<Eigen::Matrix3d>* fundamental_matrices) const {
			std::vector<Eigen::Vector2d> image1_points, image2_points;
			for (int i = 0; i < 8; i++) {
			  image1_points.emplace_back(correspondences[i].feature1);
			  image2_points.emplace_back(correspondences[i].feature2);
			}


			
			Eigen::Matrix3d fmatrix;
			if (!NormalizedEightPointFundamentalMatrix(
			        image1_points, image2_points, &fmatrix)) {
			  return false;
			}

			fundamental_matrices->emplace_back(fmatrix);
			return true;
		}

		// The error for a correspondences given a model. This is the squared sampson
		// error.
		double Error(const matching::FeatureCorrespondence& correspondence,
		           const Eigen::Matrix3d& fundamental_matrix) const {
			return SquaredSampsonDistance(fundamental_matrix,
		                              correspondence.feature1,
		                              correspondence.feature2);
  		}

  	private:
  		DISALLOW_COPY_AND_ASSIGN(FundamentalMatrixEstimator);


};

} // namespace

bool EstimateFundamentalMatrix(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<matching::FeatureCorrespondence>& normalized_correspondences,
    Eigen::Matrix3d* fundamental_matrix,
    RansacSummary* ransac_summary) {
  FundamentalMatrixEstimator fundamental_matrix_estimator;
  std::unique_ptr<SampleConsensusEstimator<FundamentalMatrixEstimator> >
      ransac = CreateAndInitializeRansacVariant(ransac_type,
                                                ransac_params,
                                                fundamental_matrix_estimator);

  // Estimate essential matrix.
  return ransac->Estimate(normalized_correspondences,
                          fundamental_matrix,
                          ransac_summary);
}



}	// namespace solvers
}	// namespace Open_SfM_METU



