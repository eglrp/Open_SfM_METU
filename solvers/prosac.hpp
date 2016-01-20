#ifndef OPEN_SFM_METU_SOLVERS_PROSAC_HPP
#define OPEN_SFM_METU_SOLVERS_PROSAC_HPP



#include <math.h>
#include <algorithm>
#include <cstdlib>
#include <vector>


#include "Open_SfM_METU/solvers/estimator.hpp"
#include "Open_SfM_METU/solvers/prosac_sampler.hpp"
#include "Open_SfM_METU/solvers/sample_consensus_estimator.hpp"


namespace Open_SfM_METU {
namespace solvers {


template <class ModelEstimator>
class Prosac : public SampleConsensusEstimator<ModelEstimator> {


	public:
		typedef typename ModelEstimator::Datum Datum;
		typedef typename ModelEstimator::Model Model;

		Prosac(const RansacParameters& ransac_params, const ModelEstimator& estimator)
		  : SampleConsensusEstimator<ModelEstimator>(ransac_params, estimator) {}
		~Prosac() {}

		bool Initialize() {
			Sampler<Datum>* prosac_sampler =
			    new ProsacSampler<Datum>(this->estimator_.SampleSize());
			return SampleConsensusEstimator<ModelEstimator>::Initialize(prosac_sampler);
		}

};


}
}






#endif // OPEN_SFM_METU_SOLVERS_PROSAC_HPP