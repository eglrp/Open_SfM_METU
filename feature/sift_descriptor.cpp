

#include "Open_SfM_METU/feature/sift_descriptor.hpp"
#include "Open_SfM_METU/image/image.hpp"

#include <algorithm>

extern "C" {
	#include <Open_SfM_METU/nonFree/vlfeat/vl/sift.h>
}

#include <vector>
#include <Eigen/Core>
#include "Open_SfM_METU/feature/descriptor_extractor.hpp"
#include "Open_SfM_METU/feature/keypoint.hpp"

#include <glog/logging.h>


namespace Open_SfM_METU {
namespace feature {


	namespace{

		// Maximum scaled dimension to extract descriptors. If the dimension is larger
		// than this then we begin to have memory and speed issues.
		static const int kMaxScaledDim = 3600;

		// Converts to a RootSIFT descriptor which is proven to provide better matches
		// for SIFT: "Three things everyone should know to improve object retrieval" by
		// Arandjelovic and Zisserman.
		void ConvertToRootSift(Eigen::VectorXf* descriptor) {
		  const double l1_norm = descriptor->lpNorm<1>();
		  *descriptor /= l1_norm;
		  *descriptor = descriptor->array().sqrt();
		}

		double GetValidFirstOctave(const int first_octave,
		                           const int width,
		                           const int height) {
		  const int max_dim = std::max(width, height);
		  int valid_first_octave = first_octave;
		  double scale_factor = std::pow(2.0, -1 * valid_first_octave);
		  while (max_dim * scale_factor >= kMaxScaledDim) {
		    scale_factor /= 2.0;
		    ++valid_first_octave;
		  }
		  return valid_first_octave;
		}

	}	// namespace



	SiftDescriptorExtractor::~SiftDescriptorExtractor() {
	  if (sift_filter_ != nullptr) vl_sift_delete(sift_filter_);
	}

	bool SiftDescriptorExtractor::ComputeDescriptor(const image::Image<unsigned char>& image,
												    const Keypoint& keypoint,
												    Eigen::VectorXf* descriptor) {
	  CHECK(keypoint.has_scale() && keypoint.has_orientation()) << "Keypoint must have scale and orientation to compute a SIFT "  << "descriptor.";
	  
	  // If the filter has been set, but is not usable for the input image (i.e. the
	  // width and height are different) then we must make a new filter. Adding this
	  // statement will save the function from regenerating the filter for
	  // successive calls with images of the same size (e.g. a video sequence).
	  if (sift_filter_ == nullptr || (sift_filter_->width != image.Width() ||
	                                  sift_filter_->height != image.Height())) {
	    vl_sift_delete(sift_filter_);
	    const int first_octave =
	        GetValidFirstOctave(sift_params_.first_octave, image.Height(), image.Width());
	    sift_filter_ = vl_sift_new(image.Width(), image.Height(),
	                               sift_params_.num_octaves,
	                               sift_params_.num_levels, first_octave);
	  }

	  // Create the vl sift keypoint from the one passed in.
	  VlSiftKeypoint sift_keypoint;
	  vl_sift_keypoint_init(sift_filter_, &sift_keypoint, keypoint.x(),
	                        keypoint.y(), keypoint.scale());

	  // The VLFeat functions take in a non-const image pointer so that it can
	  // calculate gaussian pyramids. Obviously, we do not want to break our const
	  // input, so the best solution (for now) is to copy the image.
	  
	  //FloatImage mutable_image = image.AsGrayscaleImage();
	  const image::Image<float> mutable_image(image.GetMat().cast<float>());

	  // Calculate the first octave to process.
	  int vl_status =
	      vl_sift_process_first_octave(sift_filter_, mutable_image.data());
	  // Proceed through the octaves we reach the same one as the keypoint.
	  while (sift_keypoint.o != sift_filter_->o_cur)
	    vl_sift_process_next_octave(sift_filter_);

	  if (vl_status == VL_ERR_EOF){
	    // akin LOG(FATAL) << "could not extract sift descriptors";
	  	std::cout << "could not extract sift descriptors" << std::endl;

	  }
	  
	  // Calculate the sift feature. Note that we are passing in a direct pointer to
	  // the descriptor's underlying data.
	  // akin CHECK_NOTNULL(descriptor)->resize(128);
	  (descriptor)->resize(128);
	  vl_sift_calc_keypoint_descriptor(sift_filter_, descriptor->data(),
	                                   &sift_keypoint, keypoint.orientation());
	  if (sift_params_.root_sift) {
	    ConvertToRootSift(descriptor);
	  }
	  return true;
	}


	bool SiftDescriptorExtractor::ComputeDescriptors(const image::Image<unsigned char>& image,
													 std::vector<Keypoint>* keypoints,
													 std::vector<Eigen::VectorXf>* descriptors) {
	  // If the filter has been set, but is not usable for the input image (i.e. the
	  // width and height are different) then we must make a new filter. Adding this
	  // statement will save the function from regenerating the filter for
	  // successive calls with images of the same size (e.g. a video sequence).
	  if (sift_filter_ == nullptr || (sift_filter_->width != image.Width() ||
	                                  sift_filter_->height != image.Height())) {
	    vl_sift_delete(sift_filter_);
	    const int first_octave =
	        GetValidFirstOctave(sift_params_.first_octave,
	                            image.Height(), image.Width());
	    sift_filter_ = vl_sift_new(image.Width(), image.Height(),
	                               sift_params_.num_octaves,
	                               sift_params_.num_levels, first_octave);
	  }

	  // Create the vl sift keypoint from the one passed in.
	  std::vector<VlSiftKeypoint> sift_keypoints(keypoints->size());
	  for (int i = 0; i < keypoints->size(); i++) {
	    // akin CHECK((*keypoints)[i].has_scale() && (*keypoints)[i].has_orientation()) << "Keypoint must have scale and orientation to compute a SIFT " << "descriptor.";
	    vl_sift_keypoint_init(sift_filter_,
	                          &sift_keypoints[i],
	                          (*keypoints)[i].x(),
	                          (*keypoints)[i].y(),
	                          (*keypoints)[i].scale());
	  }
	  // The VLFeat functions take in a non-const image pointer so that it can
	  // calculate gaussian pyramids. Obviously, we do not want to break our const
	  // input, so the best solution (for now) is to copy the image.
	  
	  //FloatImage mutable_image = image.AsGrayscaleImage();
	  const image::Image<float> mutable_image(image.GetMat().cast<float>());


	  // Calculate the first octave to process.
	  int vl_status =
	      vl_sift_process_first_octave(sift_filter_, mutable_image.data());

	  // Proceed through the octaves we reach the same one as the keypoint.  We
	  // first resize the descriptors vector so that the keypoint indicies will be
	  // properly matched to the descriptors.
	  descriptors->resize(keypoints->size(), Eigen::VectorXf(128));
	  while (vl_status != VL_ERR_EOF) {
	    // Go through each keypoint to see if it came from this octave.
	    for (int i = 0; i < sift_keypoints.size(); i++) {
	      if (sift_keypoints[i].o != sift_filter_->o_cur) continue;

	      vl_sift_calc_keypoint_descriptor(
	          sift_filter_, (*descriptors)[i].data(), &sift_keypoints[i],
	          (*keypoints)[i].orientation());
	      if (sift_params_.root_sift) {
	        ConvertToRootSift(&descriptors->at(i));
	      }
	    }
	    vl_status = vl_sift_process_next_octave(sift_filter_);
	  }
	  return true;
	}

	/*

	bool SiftDescriptorExtractor::DetectAndExtractDescriptors(
    const FloatImage& image,
    std::vector<Keypoint>* keypoints,
    std::vector<Eigen::VectorXf>* descriptors) {
	  // If the filter has been set, but is not usable for the input image (i.e. the
	  // width and height are different) then we must make a new filter. Adding this
	  // statement will save the function from regenerating the filter for
	  // successive calls with images of the same size (e.g. a video sequence).
	  if (sift_filter_ == nullptr || (sift_filter_->width != image.Cols() ||
	                                  sift_filter_->height != image.Rows())) {
	    vl_sift_delete(sift_filter_);
	    const int first_octave =
	        GetValidFirstOctave(sift_params_.first_octave,
	                            image.Rows(), image.Cols());
	    sift_filter_ = vl_sift_new(image.Cols(), image.Rows(),
	                               sift_params_.num_octaves,
	                               sift_params_.num_levels, first_octave);
	    vl_sift_set_edge_thresh(sift_filter_, sift_params_.edge_threshold);
	    vl_sift_set_peak_thresh(sift_filter_, sift_params_.peak_threshold);
	  }

	  // The VLFeat functions take in a non-const image pointer so that it can
	  // calculate gaussian pyramids. Obviously, we do not want to break our const
	  // input, so the best solution (for now) is to copy the image.
	  FloatImage mutable_image = image.AsGrayscaleImage();

	  // Calculate the first octave to process.
	  int vl_status =
	      vl_sift_process_first_octave(sift_filter_, mutable_image.Data());
	  // Process octaves until you can't anymore.
	  while (vl_status != VL_ERR_EOF) {
	    // Detect the keypoints.
	    vl_sift_detect(sift_filter_);

	    // Get the keypoints.
	    const VlSiftKeypoint* vl_keypoints = vl_sift_get_keypoints(sift_filter_);
	    const int num_keypoints = vl_sift_get_nkeypoints(sift_filter_);

	    for (int i = 0; i < num_keypoints; ++i) {
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

	      for (int j = 0; j < num_angles; ++j) {
	        descriptors->emplace_back(128);
	        vl_sift_calc_keypoint_descriptor(
	            sift_filter_, descriptors->back().data(), &vl_keypoints[i],
	            angles[j]);
	        if (sift_params_.root_sift) {
	          ConvertToRootSift(&descriptors->back());
	        }

	        Keypoint keypoint(vl_keypoints[i].x, vl_keypoints[i].y, Keypoint::SIFT);
	        keypoint.set_scale(vl_keypoints[i].sigma);
	        keypoint.set_orientation(angles[j]);
	        keypoints->push_back(keypoint);
	      }
	    }
	    // Attempt to process the next octave.
	    vl_status = vl_sift_process_next_octave(sift_filter_);
	  }
	  return true;
	}

	*/




}
}

