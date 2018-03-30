#pragma once
#include <Eigen/Dense>
#include <QtWidgets/QMainWindow>
#include "ui_AxPCTransform.h"

using namespace std;
using namespace Eigen;

class AxPCTransform : public QMainWindow
{
	Q_OBJECT

public:
	AxPCTransform(QWidget *parent = Q_NULLPTR);

	void ReadPointCloud(std::string filePath, std::string fileName, Vector3f &transform, Quaternionf &Q1);
public:
	bool isrgb;

private slots:
	void TransformPointClouds();
private:
	Ui::AxPCTransformClass ui;
};
