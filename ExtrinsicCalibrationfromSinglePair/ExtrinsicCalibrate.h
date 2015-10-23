#pragma once
#include <string>
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;
class ExtrinsicCalibrator
{
public:
	ExtrinsicCalibrator();
	~ExtrinsicCalibrator();
	bool Run();
private:
	bool loadCalibData(string camMatrix1Filename, string distMatrix1Filename, string camMatrix2Filename, string distMatrix2Filename);

	Mat m_camMat_1;
	Mat m_distMat_1;
	Mat m_camMat_2;
	Mat m_distMat_2;
};

ExtrinsicCalibrator::ExtrinsicCalibrator()
{
}

ExtrinsicCalibrator::~ExtrinsicCalibrator()
{
}