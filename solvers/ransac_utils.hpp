#ifndef OPEN_SFM_METU_SOLVERS_RANSAC_UTILS_HPP
#define OPEN_SFM_METU_SOLVERS_RANSAC_UTILS_HPP

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




namespace Open_SfM_METU {
namespace solvers {

struct RansacParameters {
	RansacParameters() 
	: error_thresh(-1),
        failure_probability(0.01),
        min_inlier_ratio(0),
        min_iterations(100),
        max_iterations(std::numeric_limits<int>::max()),
        use_mle(false),
        use_Tdd_test(false) {}


    // Error threshold to determin inliers for RANSAC (e.g., squared reprojection
    // error). This is what will be used by the estimator to determine inliers.
    double error_thresh;

    // The failure probability of RANSAC. Set to 0.01 means that RANSAC has a 1%
  	// chance of missing the correct pose.
  	double failure_probability;

  	// The minimal assumed inlier ratio, i.e., it is assumed that the given set
  	// of correspondences has an inlier ratio of at least min_inlier_ratio.
  	// This is required to limit the number of RANSAC iteratios.
  	double min_inlier_ratio;

  	// The minimum number of iterations required before exiting.
  	int min_iterations;

  	// Another way to specify the maximal number of RANSAC iterations. In effect,
	// the maximal number of iterations is set to min(max_ransac_iterations, T),
	// where T is the number of iterations corresponding to min_inlier_ratio.
	// This variable is useful if RANSAC is to be applied iteratively, i.e.,
	// first applying RANSAC with an min_inlier_ratio of x, then with one
	// of x-y and so on, and we want to avoid repeating RANSAC iterations.
	// However, the preferable way to limit the number of RANSAC iterations is
	// to set min_inlier_ratio and leave max_ransac_iterations to its default
	// value.
	// Per default, this variable is set to std::numeric_limits<int>::max().
	int max_iterations;


	// Instead of the standard inlier count, use the Maximum Likelihood Estimate
	// (MLE) to determine the best solution. Inliers are weighted by their error
	// and outliers count as a constant penalty.
	bool use_mle;

	// Whether to use the T_{d,d}, with d=1, test proposed in
	// Chum, O. and Matas, J.: Randomized RANSAC and T(d,d) test, BMVC 2002.
	// After computing the pose, RANSAC selects one match at random and evaluates
	// all poses. If the point is an outlier to one pose, the corresponding pose
	// is rejected. Notice that if the pose solver returns multiple poses, then
	// at most one pose is correct. If the selected match is correct, then only
	// the correct pose will pass the test. Per default, the test is disabled.
	//
	// NOTE: Not currently implemented!
	bool use_Tdd_test;

};

// A struct to hold useful outputs of Ransac-like methods.
struct RansacSummary {
  // Contains the indices of all inliers.
  std::vector<int> inliers;

  // The number of iterations performed before stopping RANSAC.
  int num_iterations;

  // The confidence in the solution.
  double confidence;
};

}
}


#endif // OPEN_SFM_METU_SOLVERS_RANSAC_UTILS_HPP
