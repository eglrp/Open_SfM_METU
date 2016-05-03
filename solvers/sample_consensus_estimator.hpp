#ifndef OPEN_SFM_METU_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_HPP
#define OPEN_SFM_METU_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_HPP



#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "Open_SfM_METU/solvers/estimator.hpp"
#include "Open_SfM_METU/solvers/inlier_support.hpp"
#include "Open_SfM_METU/solvers/mle_quality_measurement.hpp"
#include "Open_SfM_METU/solvers/quality_measurement.hpp"
#include "Open_SfM_METU/solvers/sampler.hpp"
#include "Open_SfM_METU/solvers/ransac_utils.hpp"


namespace Open_SfM_METU {
namespace solvers {



template <class ModelEstimator> 
class SampleConsensusEstimator {

	public:
		typedef typename ModelEstimator::Datum Datum;
		typedef typename ModelEstimator::Model Model;

		SampleConsensusEstimator(const RansacParameters& ransac_params, const ModelEstimator& estimator);


		virtual bool Initialize() { return true; }

		virtual ~SampleConsensusEstimator() {}

		// Computes the best-fitting model using RANSAC. Returns false if RANSAC
		// calculation fails and true (with the best_model output) if successful.
		// Params:
		//   data: the set from which to sample
		//   estimator: The estimator used to estimate the model based on the Datum
		//     and Model type
		//   best_model: The output parameter that will be filled with the best model
		//     estimated from RANSAC
		//virtual bool Estimate(const std::vector<Datum>& data,
		//                    Model* best_model,
		//                    RansacSummary* summary);


    virtual bool Estimate(const std::vector<Datum>& data,
                        Model* best_model,
                        RansacSummary* summary,
                        std::vector<double>* all_l_vals);


	protected:
		
		// This method is called from derived classes to set up the sampling scheme
		// and the method for computing inliers. It must be called by derived classes
		// unless they override the Estimate(...) method. The method for computing
		// inliers (standar inlier support or MLE) is determined by the ransac params.
		//
		// sampler: The class that instantiates the sampling strategy for this
		//   particular type of sampling consensus.


		bool Initialize(Sampler<Datum>* sampler);

		// Computes the maximum number of iterations required to ensure the inlier
		// ratio is the best with a probability corresponding to log_failure_prob.
		int ComputeMaxIterations(const double min_sample_size,
		                       const double inlier_ratio,
		                       const double log_failure_prob) const;

		// The sampling strategy.
		std::unique_ptr<Sampler<Datum> > sampler_;

		// The quality metric for the estimated model and data.
		std::unique_ptr<QualityMeasurement> quality_measurement_;

		// Ransac parameters (see above struct).
		const RansacParameters& ransac_params_;

		// Estimator to use for generating models.
		const ModelEstimator& estimator_;


};

// --------------------------- Implementation --------------------------------//
template <class ModelEstimator>
SampleConsensusEstimator<ModelEstimator>::SampleConsensusEstimator(
    const RansacParameters& ransac_params, const ModelEstimator& estimator)
    : ransac_params_(ransac_params), estimator_(estimator) {
  CHECK_GT(ransac_params.error_thresh, 0)
      << "Error threshold must be set to greater than zero";
  CHECK_LE(ransac_params.min_inlier_ratio, 1.0);
  CHECK_GE(ransac_params.min_inlier_ratio, 0.0);
  CHECK_LT(ransac_params.failure_probability, 1.0);
  CHECK_GT(ransac_params.failure_probability, 0.0);
  CHECK_GE(ransac_params.max_iterations, ransac_params.min_iterations);
}

template <class ModelEstimator>
bool SampleConsensusEstimator<ModelEstimator>::Initialize(
    Sampler<Datum>* sampler) {
  CHECK_NOTNULL(sampler);
  sampler_.reset(sampler);
  if (!sampler_->Initialize()) {
    return false;
  }

  if (ransac_params_.use_mle) {
    quality_measurement_.reset(
        new MLEQualityMeasurement(ransac_params_.error_thresh));
  } else {
    quality_measurement_.reset(
        new InlierSupport(ransac_params_.error_thresh));
  }
  return quality_measurement_->Initialize();
}

template <class ModelEstimator>
int SampleConsensusEstimator<ModelEstimator>::ComputeMaxIterations(
    const double min_sample_size,
    const double inlier_ratio,
    const double log_failure_prob) const {
  CHECK_GT(inlier_ratio, 0.0);
  if (inlier_ratio == 1.0) {
    return ransac_params_.min_iterations;
  }

  // If we use the T_{1,1} test, we have to adapt the number of samples
  // that needs to be generated accordingly since we use another
  // match for verification and a correct match is selected with probability
  // inlier_ratio.
  const double num_samples =
      ransac_params_.use_Tdd_test ? min_sample_size + 1 : min_sample_size;

  const double log_prob = log(1.0 - pow(inlier_ratio, num_samples))
      - std::numeric_limits<double>::epsilon();

  // NOTE: For very low inlier ratios the number of iterations can actually
  // exceed the maximum value for an int. We need to keep this variable as a
  // double until we do the check below against the minimum and maximum number
  // of iterations in the parameter settings.
  const double num_iterations = log_failure_prob / log_prob;

  return std::max(static_cast<double>(ransac_params_.min_iterations),
                  std::min(num_iterations,
                           static_cast<double>(ransac_params_.max_iterations)));
}

template <class ModelEstimator>
bool SampleConsensusEstimator<ModelEstimator>::Estimate(
    const std::vector<Datum>& data,
    Model* best_model,
    RansacSummary* summary,
    std::vector<double>* all_l_vals) {
  CHECK_GT(data.size(), 0)
      << "Cannot perform estimation with 0 data measurements!";
  CHECK_NOTNULL(sampler_.get());
  CHECK_NOTNULL(quality_measurement_.get());
  CHECK_NOTNULL(summary);
  CHECK_NOTNULL(best_model);

  const double log_failure_prob = log(ransac_params_.failure_probability);
  double best_cost = std::numeric_limits<double>::max();
  int max_iterations = ransac_params_.max_iterations;

  // Set the max iterations if the inlier ratio is set.
  if (ransac_params_.min_inlier_ratio > 0) {
    max_iterations = std::min(
        ComputeMaxIterations(estimator_.SampleSize(),
                             ransac_params_.min_inlier_ratio,
                             log_failure_prob),
        ransac_params_.max_iterations);
  }

  for (summary->num_iterations = 0;
       summary->num_iterations < max_iterations;
       summary->num_iterations++) {
    // Sample subset. Proceed if successfully sampled.
    std::vector<Datum> data_subset;
    if (!sampler_->Sample(data, &data_subset)) {
      continue;
    }

    // Estimate model from subset. Skip to next iteration if the model fails to
    // estimate.
    std::vector<Model> temp_models;
    

    if (!estimator_.EstimateModel(data_subset, &temp_models)) {
      continue;
    }
    

    //std::vector<double> l_values_inside;

    /*
    if (!estimator_.EstimateModel(data_subset, &temp_models, all_l_vals)) {
      continue;
    }
    */

    /*
    if (!estimator_.EstimateModel(data_subset, &temp_models, all_l_vals,ransac_params_.image_width, ransac_params_.num_pointCorr)) {
      continue;
    }
    */


    // Calculate residuals from estimated model.
    for (const Model& temp_model : temp_models) {
      const std::vector<double> residuals =
          estimator_.Residuals(data, temp_model);

      // Determine cost of the generated model.
      std::vector<int> inlier_indices;
      const double sample_cost =
          quality_measurement_->ComputeCost(residuals, &inlier_indices);
      const double inlier_ratio = static_cast<double>(inlier_indices.size()) /
                                  static_cast<double>(data.size());

      // Update best model if error is the best we have seen.
      if (sample_cost < best_cost) {
        *best_model = temp_model;
        best_cost = sample_cost;

        if (inlier_ratio <
            estimator_.SampleSize() / static_cast<double>(data.size())) {
          continue;
        }

        // A better cost does not guarantee a higher inlier ratio (i.e, the MLE
        // case) so we only update the max iterations if the number decreases.
        max_iterations = std::min(ComputeMaxIterations(estimator_.SampleSize(),
                                                       inlier_ratio,
                                                       log_failure_prob),
                                  max_iterations);

        VLOG(3) << "Inlier ratio = " << inlier_ratio
                << " and max number of iterations = " << max_iterations;
      }
    }
  }

  // Compute the final inliers for the best model.
  const std::vector<double> best_residuals =
      estimator_.Residuals(data, *best_model);
  quality_measurement_->ComputeCost(best_residuals, &summary->inliers);

  const double inlier_ratio =
      static_cast<double>(summary->inliers.size()) / data.size();
  summary->confidence =
      1.0 - pow(1.0 - pow(inlier_ratio, estimator_.SampleSize()),
                summary->num_iterations);


  return true;
}


}
}


#endif // OPEN_SFM_METU_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_HPP