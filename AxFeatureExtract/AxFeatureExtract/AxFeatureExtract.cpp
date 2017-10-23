// AxFeatureExtract.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "vector"

using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	cv::Mat rgb1 = cv::imread( "data\\2017- 4-11-15-52-47-rgb.png");
	cv::Mat rgb2 = cv::imread( "data\\2017- 4-11-15-53- 0-rgb.png");
	/*cv::Mat rgb1 = cv::imread( "G:\\SLAMDatasets\\一起做RGB-DSLAM数据\\rgb_png\\1.png");
	cv::Mat rgb2 = cv::imread( "G:\\SLAMDatasets\\一起做RGB-DSLAM数据\\rgb_png\\2.png");*/
	//cv::Mat depth1 = cv::imread( "G:\\SLAMDatasets\\一起做RGB-DSLAM数据\\depth_png\\1.png", -1);
	//cv::Mat depth2 = cv::imread( "G:\\SLAMDatasets\\一起做RGB-DSLAM数据\\depth_png\\2.png", -1);
	 
	// 声明特征提取器与描述子提取器
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _descriptor;
	
	// 构建提取器，默认两者都为sift
	// 构建sift, surf之前要初始化nonfree模块
	cv::initModule_nonfree();
	_detector = cv::FeatureDetector::create( "GridSIFT" );
	_descriptor = cv::DescriptorExtractor::create( "SIFT" );
	 
	vector< cv::KeyPoint > kp1, kp2; //关键点
	_detector->detect( rgb1, kp1 );  //提取关键点
	_detector->detect( rgb2, kp2 );
	
	cout<<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;
	    
	// 可视化， 显示关键点
	cv::Mat imgShow;
	cv::drawKeypoints( rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	cv::imshow( "keypoints", imgShow );
	cv::imwrite( "data\\keypoints.png", imgShow );
	cv::waitKey(0); //暂停等待一个按键
	   
	// 计算描述子
	cv::Mat desp1, desp2;
	_descriptor->compute( rgb1, kp1, desp1 );
	_descriptor->compute( rgb2, kp2, desp2 );
	
	// 匹配描述子
	vector< cv::DMatch > matches; 
	cv::FlannBasedMatcher matcher;
	matcher.match( desp1, desp2, matches );
	cout<<"Find total "<<matches.size()<<" matches."<<endl;
	
	// 可视化：显示匹配的特征
	cv::Mat imgMatches;
	cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
	cv::imshow( "matches", imgMatches );
	cv::imwrite( "data\\matches.png", imgMatches );
	cv::waitKey( 0 );
	
	// 筛选匹配，把距离太大的去掉
	// 这里使用的准则是去掉大于四倍最小距离的匹配
	vector< cv::DMatch > goodMatches;
	double minDis = 9999;
	for ( size_t i=0; i<matches.size(); i++ )
	{
			if ( matches[i].distance < minDis )
				minDis = matches[i].distance;
	}
	
	for ( size_t i=0; i<matches.size(); i++ )
	{
		if (matches[i].distance < 4*minDis)
			goodMatches.push_back( matches[i] );
	}

		// 显示 good matches
	cout<<"good matches="<<goodMatches.size()<<endl;
	cv::drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, imgMatches );
	cv::imshow( "good matches", imgMatches );
	cv::imwrite( "data\\good_matches.png", imgMatches );
	system("pause");
	return 0;
}

