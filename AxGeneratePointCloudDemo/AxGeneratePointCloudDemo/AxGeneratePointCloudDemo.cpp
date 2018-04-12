// AxGeneratePointCloudDemo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include <iostream>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

int _tmain(int argc, _TCHAR* argv[])
{
	// 图像矩阵
	cv::Mat rgb, depth;
	// 使用cv::imread()来读取图像
	rgb = cv::imread("./data/rgb.png");
	// rgb 图像是8UC3的彩色图像
	// depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
	depth = cv::imread("./data/depth.png", -1);

	std::string fileNameSave = "pointcloud.txt";
	FILE *fileSave = fopen(fileNameSave.c_str(), "w");
	if (fileSave == NULL)
	{
		return 0;
	}
	// 遍历深度图
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			ushort d = depth.ptr<ushort>(m)[n];
			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			float x, y, z;
			int r, g, b;
			// 计算这个点的空间坐标
			z = double(d) / camera_factor;
			x = (n - camera_cx) * z / camera_fx;
			y = (m - camera_cy) * z / camera_fy;

			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
			b = rgb.ptr<uchar>(m)[n * 3];
			g = rgb.ptr<uchar>(m)[n * 3 + 1];
			r = rgb.ptr<uchar>(m)[n * 3 + 2];

			fprintf(fileSave, "%f %f %f %d %d %d\n", x, y, z, r, g, b);
		}
	fclose(fileSave);
	system("pause");
	return 0;
}

