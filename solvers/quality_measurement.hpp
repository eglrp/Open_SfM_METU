#ifndef OPEN_SFM_METU_SOLVERS_QUALITY_MEASUREMENT_HPP
#define OPEN_SFM_METU_SOLVERS_QUALITY_MEASUREMENT_HPP


#include <vector>

namespace Open_SfM_METU {
namespace solvers {

// Purely virtual class to be used with sampling consensus estimators
// (e.g. Ransac, Prosac, MLESac, etc.). This class is implemented to assess the
// quality of the data. A trivial example is the inlier quality measurement
// (i.e. if the error for a measurement is less than a threshold, then it is an
// inlier).

class QualityMeasurement {


	public:

		QualityMeasurement() {}
		explicit QualityMeasurement(const double error_thresh)	: error_thresh_(error_thresh) {}
  		virtual ~QualityMeasurement() {}


  		// Initializes any non-trivial variables and sets up sampler if
		// necessary. Must be called before Compare is called.
		virtual bool Initialize() { return true; }

		// Given the residuals, assess a quality metric for the data. This returns a
		// cost, so lower is better. The indicies of the inliers are additionally
		// returned.
		virtual double ComputeCost(const std::vector<double>& residuals, std::vector<int>* inliers) = 0;

	protected:
  		
  		double error_thresh_;

};

}
}


#endif // OPEN_SFM_METU_SOLVERS_QUALITY_MEASUREMENT_HPP