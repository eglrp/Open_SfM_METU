#ifndef OPEN_SFM_METU_SOLVERS_RANSAC_HPP
#define OPEN_SFM_METU_SOLVERS_RANSAC_HPP




#include <math.h>
#include <cstdlib>
#include <vector>

#include "Open_SfM_METU/solvers/estimator.hpp"
#include "Open_SfM_METU/solvers/sample_consensus_estimator.hpp"
#include "Open_SfM_METU/solvers/random_sampler.hpp"


namespace Open_SfM_METU {
namespace solvers {


template <class ModelEstimator>
class Ransac : public SampleConsensusEstimator<ModelEstimator> {

	public:
		typedef typename ModelEstimator::Datum Datum;
		typedef typename ModelEstimator::Model Model;

		Ransac(const RansacParameters& ransac_params, const ModelEstimator& estimator) 
			: SampleConsensusEstimator<ModelEstimator>(ransac_params, estimator) {}
		virtual ~Ransac() {}

		// Initializes the random sampler and inlier support measurement.
		bool Initialize() {
		Sampler<Datum>* random_sampler = new RandomSampler<Datum>(this->estimator_.SampleSize());
			return SampleConsensusEstimator<ModelEstimator>::Initialize(random_sampler);
		}

};

}
}



#endif // OPEN_SFM_METU_SOLVERS_RANSAC_HPP