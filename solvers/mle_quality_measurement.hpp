#ifndef OPEN_SFM_METU_SOLVERS_MLE_QUALITY_MEASUREMENT_HPP
#define OPEN_SFM_METU_SOLVERS_MLE_QUALITY_MEASUREMENT_HPP



#include <glog/logging.h>
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "Open_SfM_METU/solvers/quality_measurement.hpp"
#include "Open_SfM_METU/solvers/distribution.hpp"



namespace Open_SfM_METU {
namespace solvers {
// Define the quality metric according to Guided MLE from "MLESAC: A new robust
// estimator with application to estimating image geometry" by Torr.

class MLEQualityMeasurement : public QualityMeasurement {

	public:
		explicit MLEQualityMeasurement(const double error_thresh) : QualityMeasurement(error_thresh) {}

		~MLEQualityMeasurement() {}

		// Given the residuals, assess a quality metric for the data. Returns the
		// quality assessment and outputs a vector of bools indicating the inliers.
		double ComputeCost(const std::vector<double>& residuals, std::vector<int>* inliers) override {
			inliers->reserve(residuals.size());
			double mle_score = 0.0;
			for (int i = 0; i < residuals.size(); i++) {
			  if (residuals[i] < error_thresh_) {
			    mle_score += residuals[i];
			    inliers->emplace_back(i);
			  } else {
			    mle_score += error_thresh_;
			  }
			}
			return mle_score;
		}

};

}
}




#endif // OPEN_SFM_METU_SOLVERS_MLE_QUALITY_MEASUREMENT_HPP