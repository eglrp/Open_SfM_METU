#ifndef OPEN_SFM_METU_SOLVERS_RANDOM_SAMPLER_HPP
#define OPEN_SFM_METU_SOLVERS_RANDOM_SAMPLER_HPP


#include <stdlib.h>
#include <algorithm>
#include <numeric>
#include <vector>


#include "Open_SfM_METU/solvers/sampler.hpp"
#include "Open_SfM_METU/solvers/random.hpp"


namespace Open_SfM_METU {
namespace solvers {


// Random sampler used for RANSAC. This is guaranteed to generate a unique
// sample by performing a Fisher-Yates sampling.
template <class Datum> class RandomSampler : public Sampler<Datum> {


	public:
	  explicit RandomSampler(const int min_num_samples)
	      : Sampler<Datum>(min_num_samples) {}
	  ~RandomSampler() {}

	  bool Initialize() override {
	    InitRandomGenerator();
	    return true;
	  }

	  // Samples the input variable data and fills the vector subset with the
	  // random samples.
	  bool Sample(const std::vector<Datum>& data,
	              std::vector<Datum>* subset) override {
	    subset->resize(this->min_num_samples_);
	    std::vector<int> random_numbers(data.size());
	    std::iota(random_numbers.begin(), random_numbers.end(), 0);

	    for (int i = 0; i < this->min_num_samples_; i++) {
	      std::swap(random_numbers[i], random_numbers[RandInt(i, data.size() - 1)]);
	      (*subset)[i] = data[random_numbers[i]];
	    }

	    return true;
	  }




};


}
}



#endif // OPEN_SFM_METU_SOLVERS_RANDOM_SAMPLER_HPP