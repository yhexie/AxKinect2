#pragma once
#include<opencv2/core/core.hpp>
#include <Eigen/Dense>
#include"../g2o/g2o/types/types_six_dof_expmap.h"
#include"../g2o/g2o/types/types_seven_dof_expmap.h"
#include <QtWidgets/QMainWindow>
#include "ui_AxPCTransform.h"

#include<System.h>
#include "Converter.h"

using namespace std;
using namespace Eigen;

class AxPCTransform : public QMainWindow
{
	Q_OBJECT

public:
	AxPCTransform(QWidget *parent = Q_NULLPTR);

	void ReadPointCloud(std::string filePath, std::string fileName, Eigen::Vector3d &transform, Eigen::Quaterniond &Q_1);
public:
	bool isrgb;

private slots:
	void TransformPointClouds();
private:
	Ui::AxPCTransformClass ui;

public:
	inline g2o::SE3Quat toSE3Quat(Eigen::Vector3d &transform, Eigen::Quaterniond &Q_1)
	{
		Eigen::Matrix<double, 3, 3> R;
		R = Q_1.toRotationMatrix();
		/*R << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2),
			cvT.at<float>(1, 0), cvT.at<float>(1, 1), cvT.at<float>(1, 2),
			cvT.at<float>(2, 0), cvT.at<float>(2, 1), cvT.at<float>(2, 2);*/

		Eigen::Matrix<double, 3, 1> t(transform.x(), transform.y(), transform.z());

		return g2o::SE3Quat(R, t);
	}
};
