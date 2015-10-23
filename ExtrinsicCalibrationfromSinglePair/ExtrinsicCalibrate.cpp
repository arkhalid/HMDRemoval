#include"ExtrinsicCalibrate.h"

ExtrinsicCalibrator::ExtrinsicCalibrator()
{
}

ExtrinsicCalibrator::~ExtrinsicCalibrator()
{
}

bool ExtrinsicCalibrator::Run()
{
	loadCalibData("../Data/camCalib/cam_rgb_1.xml", "../Data/camCalib/distort_rgb_1.xml", "../Data/camCalib/cam_rgb_2.xml", "../Data/camCalib/distort_rgb_2.xml");
	Mat img1,img2;
	imread
	return true;
}

bool ExtrinsicCalibrator::loadCalibData(string camMatrix1Filename, string distMatrix1Filename, string camMatrix2Filename, string distMatrix2Filename)
{
	CvMat* temp0, *temp1,*temp2,*temp3;
	temp0 = (CvMat*)cvLoad(camMatrix1Filename.c_str(), NULL, NULL, NULL);
	temp1 = (CvMat*)cvLoad(distMatrix1Filename.c_str(), NULL, NULL, NULL);
	temp2 = (CvMat*)cvLoad(camMatrix2Filename.c_str(), NULL, NULL, NULL);
	temp3 = (CvMat*)cvLoad(distMatrix2Filename.c_str(), NULL, NULL, NULL);
	if (temp0 == NULL || temp1 == NULL || temp2 == NULL || temp3 == NULL)
	{
		return false;
	}
	else
	{
		m_camMat_1 = cv::Mat(temp0);
		m_distMat_1 = cv::Mat(temp1);
		m_camMat_2 = cv::Mat(temp2);
		m_distMat_2 = cv::Mat(temp3);
	}
	return true;
	return false;
}
