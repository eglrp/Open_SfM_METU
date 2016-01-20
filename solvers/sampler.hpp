#ifndef OPEN_SFM_METU_SOLVERS_SAMPLER_HPP
#define OPEN_SFM_METU_SOLVERS_SAMPLER_HPP


#include <vector>

namespace Open_SfM_METU {
namespace solvers {


// Purely virtual class used for the sampling consensus methods (e.g. Ransac,
// Prosac, MLESac, etc.)
template <class Datum> class Sampler {
 public:
  explicit Sampler(const int min_num_samples)
      : min_num_samples_(min_num_samples) {}

  // Initializes any non-trivial variables and sets up sampler if
  // necessary. Must be called before Sample is called.
  virtual bool Initialize() = 0;

  virtual ~Sampler() {}
  // Samples the input variable data and fills the vector subset with the
  // samples.
  virtual bool Sample(const std::vector<Datum>& data,
                      std::vector<Datum>* subset) = 0;

 protected:
  int min_num_samples_;



};


}
}



#endif // OPEN_SFM_METU_SOLVERS_SAMPLER_HPP