#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <vector>
#include <sstream>
using namespace std;
class MultiCameraCaptureApp
{
public:
	MultiCameraCaptureApp();
	~MultiCameraCaptureApp();
	bool Run(int argc, char * argv[]);
private:
	struct camInfo
	{
		cv::VideoCapture cap;
		int imgWidth;
		int imgHeight;
		cv::Mat img;
	};
	bool init();
	bool initCams();

	vector<camInfo> m_camList;
	int m_counter;
	bool m_saveImage;
};
