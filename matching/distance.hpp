#ifndef OPEN_SFM_METU_DISTANCE_HPP
#define OPEN_SFM_METU_DISTANCE_HPP


#include <Eigen/Core>
#include <glog/logging.h>


namespace Open_SfM_METU {
namespace matching {


// This file includes all of the distance metrics that are used:
// L2 distance for euclidean features.

// Squared Euclidean distance functor. We let Eigen handle the SSE optimization.
// NOTE: This assumes that each vector has a unit norm:
//  ||x - y||^2 = ||x||^2 + ||y||^2 - 2*||x^t * y|| = 2 - 2 * x.dot(y).
struct L2 {
  typedef float DistanceType;
  typedef Eigen::VectorXf DescriptorType;

  DistanceType operator()(const Eigen::VectorXf& descriptor_a,
                          const Eigen::VectorXf& descriptor_b) const {
    DCHECK_EQ(descriptor_a.size(), descriptor_b.size());
    const DistanceType dist = 2.0 - 2.0 * descriptor_a.dot(descriptor_b);
    return dist;
  }
};


}
}


#endif // OPEN_SFM_METU_DISTANCE_HPP