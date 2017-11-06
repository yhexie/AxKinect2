#pragma once
#ifndef AXPAIRWISEREGISTRATION_HEADER
#define AXPAIRWISEREGISTRATION_HEADER
#include "stdafx.h"
#include <iostream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>
#include "Camera_Intrinsic_Parameters.h"

using namespace cv;
using namespace Eigen;
enum RESULT_MATCH
{
	NOT_MATCH = 0,
	MATCH = 1
};
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
		p.y = (point.y - camera.camera_cy)*p.z / camera.camera_fy;
		return p;
	}
	double normOfTransform(cv::Mat rvec, cv::Mat tvec)
	{
		return abs(min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec))) + abs(cv::norm(tvec));
	}

	void transformPointcloud(const Point3f cloud_in, Point3f& cloud_out,const Eigen::Matrix<double, 4, 4>  &transform);

	int PnPMatch();

	Eigen::Matrix4d getTransformation()
	{
		return transformation;
	}

	double getAbsMotionDistance()
	{
		return distance;
	}

	// cvMat2Eigen
	Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec)
	{
		cv::Mat R;
		cv::Rodrigues(rvec, R);
		Eigen::Matrix3d r;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				r(i, j) = R.at<double>(i, j);

		// 将平移向量和旋转矩阵转换成变换矩阵
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

		Eigen::AngleAxisd angle(r);
		T = angle;
		T(0, 3) = tvec.at<double>(0, 0);
		T(1, 3) = tvec.at<double>(1, 0);
		T(2, 3) = tvec.at<double>(2, 0);
		return T;
	}
public:
	cv::Mat source_rgb;
	cv::Mat target_rgb;
	cv::Mat source_depth;
	cv::Mat target_depth;
	Camera_Intrinsic_Parameters depth_intrinsic_par;

	Eigen::Matrix4d transformation;
	double distance;
};
#endif // !AXPAIRWISEREGISTRATION_HEADER

