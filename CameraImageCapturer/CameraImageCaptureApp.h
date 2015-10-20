#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <sstream>
#include <string>

using namespace std;
class CameraImageCaptureApp
{
public:
	CameraImageCaptureApp();
	~CameraImageCaptureApp();
	bool Run(int argc, char* argv[]);
private:
	bool init();
	bool initCamera(int idx);
	bool loadCalibData(string camMatrixFilename, string distMatrixFilename);
	cv::VideoCapture m_cap;
	cv::Mat m_img;
	int m_imgWidth;
	int m_imgHeight;
	bool m_loadCalib;
	cv::Mat m_camMat;
	cv::Mat m_distMat;
	bool m_saveImage;
	int m_counter;
};

