#ifndef CAMERA_INTRINSIC_PARAMETERS_HEADER
#define CAMERA_INTRINSIC_PARAMETERS_HEADER
//相机内参

class Camera_Intrinsic_Parameters
{
public:
	float camera_cx, camera_cy, camera_fx, camera_fy, camera_factor;
public:
	Camera_Intrinsic_Parameters(void)
	{

	}
	Camera_Intrinsic_Parameters(float cx, float cy, float fx, float fy, float scale)
	{
		camera_cx = cx;
		camera_cy = cy;
		camera_fx = fx;
		camera_fy = fy;
		camera_factor = scale;
	}
};

struct Result_of_PNP
{
	cv::Mat rvec, tvec;
	int inliers;
};
#endif