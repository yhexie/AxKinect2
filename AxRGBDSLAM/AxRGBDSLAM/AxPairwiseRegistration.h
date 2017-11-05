#pragma once
#ifndef AXPAIRWISEREGISTRATION_HEADER
#define AXPAIRWISEREGISTRATION_HEADER

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>
#include "Camera_Intrinsic_Parameters.h"

using namespace cv;
using namespace Eigen;
class AxPairwiseRegistration
{
public:
	AxPairwiseRegistration();
	~AxPairwiseRegistration();
	void setSourceRGB(cv::Mat source_);
	void setTargetRGB(cv::Mat target_);
	void setSourceDepth(cv::Mat sourced_);
	void setTargetDepth(cv::Mat targetd_);
	void setDepthIntrinsicParams(Camera_Intrinsic_Parameters params);

	cv::Point3f point2dTo3d(cv::Point3f & point, Camera_Intrinsic_Parameters camera)
	{
		cv::Point3f p;
		p.z = point.z / camera.camera_factor;
		p.x = (point.x - camera.camera_cx)*p.z / camera.camera_fx;
		p.y = (point.y - camera.camera_fy)*p.z / camera.camera_fy;
		return p;
	}
	void PnPMatch();

	Eigen::Matrix4d getTransformation()
	{
		return transformation;
	}
public:
	cv::Mat source_rgb;
	cv::Mat target_rgb;
	cv::Mat source_depth;
	cv::Mat target_depth;
	Eigen::Matrix4d transformation;
	Camera_Intrinsic_Parameters depth_intrinsic_par;
};
#endif // !AXPAIRWISEREGISTRATION_HEADER

