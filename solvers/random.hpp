#ifndef OPEN_SFM_METU_SOLVERS_RANDOM_HPP
#define OPEN_SFM_METU_SOLVERS_RANDOM_HPP

namespace Open_SfM_METU {
namespace solvers {


// Initializes the random generator to be based on the current time. Does not
// have to be called before calling RandDouble, but it works best if it is.
void InitRandomGenerator();

// Get a random double between lower and upper (inclusive).
double RandDouble(double lower, double upper);

// Get a random double between lower and upper (inclusive).
int RandInt(int lower, int upper);

// Generate a number drawn from a gaussian distribution.
double RandGaussian(double mean, double std_dev);



}
}



#endif // OPEN_SFM_METU_SOLVERS_RANDOM_HPP