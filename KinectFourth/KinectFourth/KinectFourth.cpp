// KinectFirst.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>

//using Eigen::MatrixXd;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using namespace cv;
using namespace std;

// 安全释放指针
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

const double camera_factor = 1000;
const double camera_cx = 263.73696;
const double camera_cy = 201.72450;
const double camera_fx = 379.40726;
const double camera_fy = 378.54472;

int _tmain(int argc, _TCHAR* argv[])
{
	//相机内参
	Eigen::Matrix3d intricRGB;
	intricRGB << 1094.75283, 0, 942.00992,
				 0, 1087.37528, 530.35240,
				 0, 0, 1;
	Eigen::Matrix3d intricDepth;
	intricDepth << camera_fx, 0, camera_cx,
				   0, camera_fy, camera_cy,
		           0, 0, 1;
	Eigen::Matrix3d intricDepth2RGB;
	intricDepth2RGB = intricRGB*intricDepth.inverse();

	Eigen::Matrix3d intricRGB2Depth;
	intricRGB2Depth = intricDepth*intricRGB.inverse();
	// 获取Kinect设备
	IKinectSensor* m_pKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* m_pMultiFrameReader=NULL;
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// 获取多数据源到读取器  
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared |
				FrameSourceTypes::FrameSourceTypes_Depth,
				&m_pMultiFrameReader);
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	// 三个数据帧及引用
	IDepthFrameReference* m_pDepthFrameReference = NULL;
	IColorFrameReference* m_pColorFrameReference = NULL;
	IInfraredFrameReference* m_pInfraredFrameReference = NULL;
	IInfraredFrame* m_pInfraredFrame = NULL;
	IDepthFrame* m_pDepthFrame = NULL;
	IColorFrame* m_pColorFrame = NULL;
	// 三个图片格式
	Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat i_depth(424, 512, CV_16UC1);
	Mat i_ir(424, 512, CV_16UC1);
	Mat result(424, 512, CV_8UC4);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	// 声明特征提取器与描述子提取器
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _descriptor;

	// 构建提取器，默认两者都为sift
	// 构建sift, surf之前要初始化nonfree模块
	cv::initModule_nonfree();
	_detector = cv::FeatureDetector::create("GridSIFT");
	_descriptor = cv::DescriptorExtractor::create("SIFT");

	vector< cv::KeyPoint > kp1, kp2; //关键点
	

	while (true)
	{
		// 获取新的一个多源数据帧
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
			//cout << "!!!" << endl;
			continue;
		}

		// 从多源数据帧中分离出彩色数据，深度数据和红外数据
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);

		// color拷贝到图片中
		UINT nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

		// depth拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
			for (int i = 0; i < 512 * 424; i++)
			{
				// 0-255深度图，为了显示明显，只取深度数据的低8位
				UINT16 intensity = static_cast<UINT16>(depthData[i]);
				reinterpret_cast<UINT16*>(i_depth.data)[i] = intensity;
			}

			// 实际是16位unsigned int数据
			//hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
		}

		// infrared拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
		}

		
		//int i = 0;
		//for (int row = 0; row < 424; row++)
		//{
		//	for (int col = 0; col < 512; col++)
		//	{
		//		UINT16* p = (UINT16*)i_depth.data;
		//		UINT16 depthValue = static_cast<UINT16>(p[row * 512 + col]);
		//		//cout << "depthValue       " << depthValue << endl;
		//		if (depthValue != -std::numeric_limits<UINT16>::infinity() && depthValue != -std::numeric_limits<UINT16>::infinity() && depthValue != 0 && depthValue != 65535)
		//		{
		//			// 投影到彩色图上的坐标
		//			Eigen::Vector3d uv_depth(col, row, 1.0f);
		//			Eigen::Vector3d uv_color = (double)depthValue * intricDepth2RGB * uv_depth;//  / 1000.f +T / 1000;

		//			int X = static_cast<int>(uv_color[0] / uv_color[2]);
		//			int Y = static_cast<int>(uv_color[1] / uv_color[2]);
		//			//cout << "X:       " << X << "     Y:      " << Y << endl;
		//			if ((X >= 0 && X < 1920) && (Y >= 0 && Y <1080 ))
		//			{
		//				//cout << "X:       " << X << "     Y:      " << Y << endl;
		//				result.data[i * 4] = i_rgb.data[4 * (Y * 1920 + X)];
		//				result.data[i * 4 + 1] = i_rgb.data[4 * (Y * 1920 + X) + 1];
		//				result.data[i * 4 + 2] = i_rgb.data[4 * (Y * 1920 + X) + 2];
		//				result.data[i * 4 + 3] = i_rgb.data[4 * (Y * 1920 + X) + 3];
		//				
		//			}
		//		}
		//		i++;
		//	}
		//}
		int icor = 0;
		for (int row = 0; row < 1080; row++)
		{
			for (int col = 0; col < 1920; col++)
			{
				/*UINT16* p = (UINT16*)i_rgb.data;
				UINT16 r = static_cast<UINT16>(p[4*(row * 1920 + col)]);
				UINT16 g = static_cast<UINT16>(p[4 * (row * 1920 + col)+1]);
				UINT16 b = static_cast<UINT16>(p[4 * (row * 1920 + col)+2]);
				UINT16 a = static_cast<UINT16>(p[4 * (row * 1920 + col) + 3]);*/
				//cout << "depthValue       " << depthValue << endl;
				//if (depthValue != -std::numeric_limits<UINT16>::infinity() && depthValue != -std::numeric_limits<UINT16>::infinity() && depthValue != 0 && depthValue != 65535)
				{
					// 彩色图投影到深度图上的坐标
					Eigen::Vector3d uv_color(col, row, 1.0f);
					Eigen::Vector3d uv_depth = intricRGB2Depth* uv_color;//  / 1000.f +T / 1000;

					int X = static_cast<int>(uv_depth[0] / uv_depth[2]);
					int Y = static_cast<int>(uv_depth[1] / uv_depth[2]);
					//cout << "X:       " << X << "     Y:      " << Y << endl;
					if ((X >= 0 && X < 512) && (Y >= 0 && Y < 424))
					{
						//cout << "X:       " << X << "     Y:      " << Y << endl;
						result.data[4 * (Y * 512 + X)] = i_rgb.data[4 * icor];
						result.data[4 * (Y * 512 + X) + 1] = i_rgb.data[4 * icor + 1];
						result.data[4 * (Y * 512 + X) + 2] = i_rgb.data[4 * icor + 2];
						result.data[4 * (Y * 512 + X) + 3] = i_rgb.data[4 * icor + 3];

					}
				}
				icor++;
			}
		}
		// 显示
		//imshow("rgb", i_rgb);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;
		imshow("depth", i_depth);
		imwrite("data\\depth.png", i_depth);
		if (waitKey(1) == VK_ESCAPE)
			break;

		imshow("mosic", result);
		imwrite("data\\img.png", result);
		if (waitKey(1) == VK_ESCAPE)
			break;

		_detector->detect(result, kp1);  //提取关键点

		cv::Mat image;
		image.create(result.rows, result.cols, CV_8UC3);
		//src.copyTo(image);  
		int fromTo1[] = { 0, 0, 1, 1, 2, 2 };
		mixChannels(&result, 1, &image, 1, fromTo1, 3);
		cv::drawKeypoints(result, kp1, image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::imshow("keypoints", image);
		cv::imwrite("data\\keypoints.png", image);
		
		/*imshow("ir", i_ir);
		if (waitKey(1) == VK_ESCAPE)
			break;*/
		if (waitKey(100) == VK_SPACE)
		{
			SYSTEMTIME st;
			GetLocalTime(&st);
			char output_file[32];
			char output_RGB[32];
			char output_depth[32];
			char output_ir[32];
			sprintf_s(output_file, "%4d-%2d-%2d-%2d-%2d-%2d.txt", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
			sprintf_s(output_RGB, "%4d-%2d-%2d-%2d-%2d-%2d-rgb.png", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
			sprintf_s(output_depth, "%4d-%2d-%2d-%2d-%2d-%2d-depth.png", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
			sprintf_s(output_ir, "%4d-%2d-%2d-%2d-%2d-%2d-ir.png", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
			FILE *file = fopen(output_file, "w");
			int idx = 0;
			for (int row = 0; row < 424; row++)
			{
				for (int col = 0; col < 512; col++)
				{
					UINT16* p = (UINT16*)i_depth.data;
					UINT16 depthValue = static_cast<UINT16>(p[row * 512 + col]);
					
					if (depthValue != -std::numeric_limits<UINT16>::infinity() && depthValue != 0)
					{
						double z = double(depthValue) / camera_factor;
						double x = (col - camera_cx) * z / camera_fx;
						double y = (row - camera_cy) * z / camera_fy;

  						float cameraX = -static_cast<float>(x);
						float cameraY = static_cast<float>(z);
						float cameraZ = -static_cast<float>(y);
						if (file)
						{
							int b = result.data[idx * 4 + 0];
							int g = result.data[idx * 4 + 1];
							int r = result.data[idx * 4 + 2];
   							fprintf(file, "%.4f %.4f %.4f %d %d %d\n", cameraX, cameraY, cameraZ, r, g, b);
						}
					}
					idx++;
				}
			}
			fclose(file);
			cout << "文件保存成功" << endl;
		}
		// 释放资源
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pInfraredFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pInfraredFrameReference);
		SafeRelease(m_pMultiFrame);
	}
	// 关闭窗口，设备
	cv::destroyAllWindows();
	m_pKinectSensor->Close();
	std::system("pause");

	return 0;
}

