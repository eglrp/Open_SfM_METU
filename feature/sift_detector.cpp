

#include "Open_SfM_METU/feature/sift_detector.hpp"
#include "Open_SfM_METU/image/image.hpp"




extern "C" {
	#include <openMVG_dependencies/nonFree/sift/vl/sift.h>
}



namespace Open_SfM_METU {
namespace feature {

	SiftDetector::~SiftDetector() {
	  if (sift_filter_ != nullptr)
	    vl_sift_delete(sift_filter_);
	}

	bool SiftDetector::DetectKeypoints(const image::Image<unsigned char>& image, std::vector<Keypoint>* keypoints) {

		// If the filter has been set, but is not usable for the input image (i.e. the
		// width and height are different) then we must make a new filter. Adding this
		// statement will save the function from regenerating the filter for
		// successive calls with images of the same size (e.g. a video sequence).
		
		
		if (sift_filter_ == nullptr || (sift_filter_->width != image.Width() ||  sift_filter_->height != image.Height())) {
			vl_sift_delete(sift_filter_);
			sift_filter_ = vl_sift_new(image.Width(), image.Height(),
			                           sift_params_.num_octaves,
			                           sift_params_.num_levels,
			                           sift_params_.first_octave);
			vl_sift_set_edge_thresh(sift_filter_, sift_params_.edge_threshold);
			vl_sift_set_peak_thresh(sift_filter_, sift_params_.peak_threshold);
		}

		
		/*
		// The VLFeat functions take in a non-const image pointer so that it can
		// calculate gaussian pyramids. Obviously, we do not want to break our const
		// input, so the best solution (for now) is to copy the image.
		FloatImage mutable_image(image.AsGrayscaleImage());

		

		// Calculate the first octave to process.
		int vl_status = vl_sift_process_first_octave(sift_filter_,
		                                           mutable_image.Data());
		// Reserve an amount that is slightly larger than what a typical detector
		// would return.
		keypoints->reserve(2000);

		// Process octaves until you can't anymore.
		while (vl_status != VL_ERR_EOF) {
		// Detect the keypoints.
		vl_sift_detect(sift_filter_);
		// Get the keypoints.
		const VlSiftKeypoint* vl_keypoints = vl_sift_get_keypoints(sift_filter_);
		int num_keypoints = vl_sift_get_nkeypoints(sift_filter_);

		for (int i = 0; i < num_keypoints; i++) {
		  // Calculate (up to 4) orientations of the keypoint.
		  double angles[4];
		  int num_angles = vl_sift_calc_keypoint_orientations(sift_filter_,
		                                                      angles,
		                                                      &vl_keypoints[i]);
		  // If upright sift is enabled, only use the first keypoint at a given
		  // pixel location.
		  if (sift_params_.upright_sift && num_angles > 1) {
		    num_angles = 1;
		  }
		  for (int j = 0; j < num_angles; j++) {
		    Keypoint keypoint(vl_keypoints[i].x, vl_keypoints[i].y, Keypoint::SIFT);
		    keypoint.set_scale(vl_keypoints[i].sigma);
		    keypoint.set_orientation(angles[j]);
		    keypoints->push_back(keypoint);
		  }
		}
		// Attempt to process the next octave.
		vl_status = vl_sift_process_next_octave(sift_filter_);
		}

		*/

		return true;


	}

}
}
