#include "AxPCTransform.h"

AxPCTransform::AxPCTransform(QWidget *parent)
	: QMainWindow(parent)
	, isrgb(true)
{
	ui.setupUi(this);
	QObject::connect(ui.action_transformPc, SIGNAL(triggered()), this, SLOT(TransformPointClouds()));
}

void AxPCTransform::TransformPointClouds()
{
	//std::string keyFramePath = "E:/PointCloudData/520fzOrbSLAM/";
	std::string keyFramePath = "E:/PointCloudData/520sss/";
	std::string keyFrameFilename="KeyFrameTrajectory.txt";
	std::string Trajactry = keyFramePath + keyFrameFilename;
	//读取关键帧位姿文件
	FILE *file = fopen(Trajactry.c_str(), "r");
	if (file == NULL)
	{
		return;
	}
	double frameId;
	float x, y, z, a, b, c,w;
	//x,y,z,四元数
	fscanf(file, "%lf %f %f %f %f %f %f %f", &frameId, &x, &y, &z, &a, &b, &c, &w);
	while (feof(file) == 0) /*判断是否文件尾，不是则循环*/
	{
		fscanf(file, "%lf %f %f %f %f %f %f %f", &frameId, &x, &y, &z, &a, &b, &c, &w);
		char astr[20];
		sprintf(astr, "%8.6lf", frameId);
		std::string aas = astr;
		std::string fileNameFrame = aas + ".txt";
		Eigen::Quaterniond Q1(w, a, b, c); // cv::Mat Rcw.t();
		Eigen::Matrix3d R1 = Q1.toRotationMatrix();
		Eigen::Matrix3d R2 = R1.transpose();//Rcw, cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
		Eigen::Quaterniond Q2(R2);
		Eigen::Vector3d transf(x, y, z);//Ow, Ow = -Rwc*tcw;	
		Eigen::Vector3d tcw = -R1.inverse()*transf;
		//传入需要转换的点云，和四元数, x, y, z, a, b, c, d
		ReadPointCloud(keyFramePath, fileNameFrame, tcw, Q2);
	}
	fclose(file);
	//
}

void AxPCTransform::ReadPointCloud(std::string filePath, std::string fileName, Eigen::Vector3d& transform, Eigen::Quaterniond &Q_1)
{
	std::string fileNameSave = filePath+"fusion_" + fileName;
	std::string fileNameOpen = filePath +  fileName;
	FILE *file = fopen(fileNameOpen.c_str(), "r");
	if (file == NULL)
	{
		return;
	}
	FILE *fileSave = fopen(fileNameSave.c_str(), "w");
	if (fileSave == NULL)
	{
		return;
	}
	Eigen::Isometry3d T = toSE3Quat(transform,Q_1);
	/*Eigen::Isometry3f T = (Eigen::Isometry3f) Q_1;
	T.translation() = transform;*/

	Vector4d coord;
	float x, y, z;
	int  a, b, c;
	if (!isrgb)
	{
		fscanf(file, "%f %f %f", &x, &y, &z);
	}
	else
	{
		fscanf(file, "%f %f %f %d %d %d", &x, &y, &z, &a, &b, &c);
		float cameraX = static_cast<float>(x);
		float cameraY = static_cast<float>(y);
		float cameraZ = static_cast<float>(z);
		coord << cameraX, cameraY, cameraZ,1;   //默认的向量为列向量
		//coord = V5.matrix()*coord + (-1)*transform;
		coord = T.inverse().matrix()*coord;
		fprintf(fileSave, "%f %f %f %d %d %d\n", coord.x(), coord.y(),coord.z(), a, b, c);
	}

	while (feof(file) == 0) /*判断是否文件尾，不是则循环*/
	{

		if (!isrgb)
		{
			fscanf(file, "%f %f %f", &x, &y, &z);
		}
		else
		{
			fscanf(file, "%f %f %f %d %d %d", &x, &y, &z, &a, &b, &c);
			float cameraX = static_cast<float>(x);
			float cameraY = static_cast<float>(y);
			float cameraZ = static_cast<float>(z);
			coord << cameraX, cameraY, cameraZ, 1;   //默认的向量为列向量
			coord = T.inverse().matrix()*coord;
			fprintf(fileSave, "%f %f %f %d %d %d\n", coord.x(), coord.y(),coord.z(), a, b, c);
		}
	}
	fclose(file);
	fclose(fileSave);
}
