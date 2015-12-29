#ifndef OPEN_SFM_METU_FEATURE_KEYPOINT_HPP
#define OPEN_SFM_METU_FEATURE_KEYPOINT_HPP



#include <vector>

namespace Open_SfM_METU {
namespace feature {


#define OPEN_SFM_METU_INVALID_KEYPOINT_VAR -9999

class Keypoint{

	public:
		enum KeypointType {
		    INVALID = -1,
		    OTHER = 0,
		    SIFT,
	  	};

		Keypoint(double x, double y, KeypointType type)
			: x_(x), y_(y), keypoint_type_(type),
		        strength_(OPEN_SFM_METU_INVALID_KEYPOINT_VAR),
		        scale_(OPEN_SFM_METU_INVALID_KEYPOINT_VAR),
		        orientation_(OPEN_SFM_METU_INVALID_KEYPOINT_VAR) {}  	

		Keypoint() : Keypoint(OPEN_SFM_METU_INVALID_KEYPOINT_VAR,
                        OPEN_SFM_METU_INVALID_KEYPOINT_VAR,
                        Keypoint::INVALID) {}
		~Keypoint() {}

		// Keypoint type.
		inline KeypointType keypoint_type() const { return keypoint_type_; }
		inline void set_keypoint_type(KeypointType type) { keypoint_type_ = type; }

		// Variable x.
		inline double x() const { return x_; }
		inline void set_x(double x) { x_ = x; }

		// Variable y.
		inline double y() const { return y_; }
		inline void set_y(double y) { y_ = y; }

		// Optional variable strength.
		inline bool has_strength() const {
		return strength_ != OPEN_SFM_METU_INVALID_KEYPOINT_VAR; }
		inline double strength() const { return strength_; }
		inline void set_strength(double strength) { strength_ = strength; }

		// Optional variable scale.
		inline bool has_scale() const { return scale_ != OPEN_SFM_METU_INVALID_KEYPOINT_VAR; }
		inline double scale() const { return scale_; }
		inline void set_scale(double scale) { scale_ = scale; }

		// Optional variable orientation.
		inline bool has_orientation() const {
		return orientation_ != OPEN_SFM_METU_INVALID_KEYPOINT_VAR; }
		inline double orientation() const { return orientation_; }
		inline void set_orientation(double orientation) {
		orientation_ = orientation; }

	private:
		double x_;
		double y_;
		KeypointType keypoint_type_;
		double strength_;
		double scale_;
		double orientation_;      
		  
};

}

}

#endif // OPEN_SFM_METU_FEATURE_KEYPOINT_HPP
