



#include "Open_SfM_METU/solvers/sequential_probability_ratio.hpp"

#include <cmath>

namespace Open_SfM_METU {
namespace solvers {

	double CalculateSPRTDecisionThreshold(double sigma, double epsilon,
	                                      double time_compute_model_ratio,
	                                      int num_models_verified) {
	  // Eq. 2 in Matas et. al.
	  double c = (1.0 - sigma) * log((1.0 - sigma) / (1.0 - epsilon)) +
	             sigma * log(sigma / epsilon);

	  // Eq. 6 in Matas et. al.
	  double a_0 = time_compute_model_ratio * c /
	                   static_cast<double>(num_models_verified) + 1.0;
	  double decision_threshold = a_0;
	  double kConvergence = 1e-4;
	  // Matas et. al. says the decision threshold typically converges in 4
	  // iterations. Set the max iters to 1000 as a safeguard, but test for
	  // convergence.
	  for (int i = 0; i < 1000; i++) {
	    double new_decision_threshold = a_0 + log(decision_threshold);
	    double step_difference = fabs(new_decision_threshold - decision_threshold);
	    decision_threshold = new_decision_threshold;
	    // If the series has converged, break.
	    if (step_difference < kConvergence) break;
	  }
	  return decision_threshold;
	}

	bool SequentialProbabilityRatioTest(const std::vector<double>& residuals,
	                                    double error_thresh, double sigma,
	                                    double epsilon, double decision_threshold,
	                                    int* num_tested_points,
	                                    double* observed_inlier_ratio) {
	  int observed_num_inliers = 0;
	  double likelihood_ratio = 1.0;
	  for (int i = 0; i < residuals.size(); i++) {
	    // Check whether i-th data point is consistent with the model. Update the
	    // likelihood ratio accordingly.
	    if (residuals[i] < error_thresh) {
	      likelihood_ratio *= sigma / epsilon;
	      observed_num_inliers += 1;
	    } else {
	      likelihood_ratio *= (1.0 - sigma) / (1.0 - epsilon);
	    }

	    // If likehood ratio exceeds our decision threshold we can terminate early.
	    if (likelihood_ratio > decision_threshold) {
	      *observed_inlier_ratio = static_cast<double>(observed_num_inliers) /
	                               static_cast<double>(i + 1);
	      *num_tested_points = i + 1;
	      return false;
	    }
	  }

	  *observed_inlier_ratio = static_cast<double>(observed_num_inliers) /
	                           static_cast<double>(residuals.size());
	  *num_tested_points = residuals.size();
	  return true;
	}


}
}
