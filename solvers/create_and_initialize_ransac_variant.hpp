#ifndef OPEN_SFM_METU_SOLVERS_HPP
#define OPEN_SFM_METU_SOLVERS_HPP


#include <glog/logging.h>

#include "Open_SfM_METU/solvers/ransac.hpp"
#include "Open_SfM_METU/solvers/arrsac.hpp"
#include "Open_SfM_METU/solvers/prosac.hpp"
#include "Open_SfM_METU/solvers/lmed.hpp"

#include "Open_SfM_METU/solvers/sample_consensus_estimator.hpp"





namespace Open_SfM_METU {
namespace solvers {


// NOTE: Prosac requires correspondences to be sorted by the descriptor
// distances with the best match first. See theia/solvers for more information
// on the various types.

enum class RansacType {
  RANSAC = 0,
  PROSAC = 1,
  // TODO(cmsweeney): Arrsac does not seem to work very fast...
  // Verify how slow it is
  ARRSAC = 2,
  LMED = 3
};

// Factory method to create a ransac variant based on the specified options. The
// variante is then initialized and fails if initialization is not successful.

template <class Estimator>
std::unique_ptr<SampleConsensusEstimator<Estimator> >
CreateAndInitializeRansacVariant(
    const RansacType& ransac_type,
    const RansacParameters& ransac_options, const Estimator& estimator) {
  std::unique_ptr<SampleConsensusEstimator<Estimator> > ransac_variant;
  switch (ransac_type) {
    case RansacType::RANSAC:
      ransac_variant.reset(new Ransac<Estimator>(ransac_options, estimator));
      break;
    case RansacType::PROSAC:
      ransac_variant.reset(new Prosac<Estimator>(ransac_options, estimator));
      break;
    case RansacType::ARRSAC:
      ransac_variant.reset(new Arrsac<Estimator>(ransac_options, estimator));
      break;
    case RansacType::LMED:
      ransac_variant.reset(new LMed<Estimator>(ransac_options, estimator));
      break;
    default:
      ransac_variant.reset(new Ransac<Estimator>(ransac_options, estimator));
      break;
  }

  CHECK(ransac_variant->Initialize()) << "Could not initialize ransac "
                                         "estimator for estimating two view "
                                         "reconstructions";
  return ransac_variant;
}



}
}



#endif // OPEN_SFM_METU_SOLVERS_HPP