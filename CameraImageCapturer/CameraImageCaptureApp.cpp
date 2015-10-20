#include "CameraImageCaptureApp.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>


bool CameraImageCaptureApp::Run(int argc, char * argv[])
{
	if (!init())
	{
		return false;
	}
	while (1)
	{
		cv::Mat undistImg= cv::Mat::zeros(m_imgHeight,m_imgWidth,CV_8UC4);
		m_cap >> m_img;
		int key;
		cv::imshow("Image", m_img);
		cv::undistort(m_img, undistImg, m_camMat, m_distMat);
		cv::imshow("Undistorted Image", undistImg);
		key = cv::waitKey(30);
		if (key == 's')
		{
			m_saveImage = true;
		}
		if (m_saveImage)
		{
			stringstream ss;
			ss << "./image_" << m_counter<<".png";
			cv::imwrite(ss.str(), undistImg);
			ss.clear();
			m_counter++;
			m_saveImage = false;
		}
	}
	return true;
}

bool CameraImageCaptureApp::init()
{
	if (m_loadCalib)
	{
		if (!loadCalibData("../Data/camCalib/cam_rgb_1.xml", "../Data/camCalib/distort_rgb_1.xml"))
		{
			std::cout << "Could not load calibrations" << std::endl;
			return false;
		}
	}
	if(!initCamera(1))
	{ 
		return false;
	}
	m_img = cv::Mat::zeros(m_imgHeight,m_imgWidth,CV_8UC4);
	
	return true;
}

bool CameraImageCaptureApp::initCamera(int idx)
{
	m_cap.open(idx);
	if (!m_cap.isOpened())
	{
		std::cout << "Could not open camera stream for camera " << idx << std::endl;
		return false;
	}
	else
	{
		m_imgWidth = m_cap.get(CV_CAP_PROP_FRAME_WIDTH);
		m_imgHeight = m_cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		return true;
	}
}

bool CameraImageCaptureApp::loadCalibData(string camMatrixFilename, string distMatrixFilename)
{
	CvMat* temp0, *temp1;
	temp0 = (CvMat*)cvLoad(camMatrixFilename.c_str(), NULL, NULL, NULL);
	temp1 = (CvMat*)cvLoad(distMatrixFilename.c_str(), NULL, NULL, NULL);
	if(temp0 == NULL || temp1 == NULL)
	{
		return false;
	}
	else
	{
		m_camMat = cv::Mat(temp0);
		m_distMat = cv::Mat(temp1);
	}
	
	return true;
}

CameraImageCaptureApp::CameraImageCaptureApp() 
	:m_loadCalib(true), m_saveImage(false), m_counter(0)
{
}

CameraImageCaptureApp::~CameraImageCaptureApp()
{
	m_cap.release();
}

CameraImageCaptureApp* mainApp;
int main(int argc, char* argv[])
{
	mainApp = new CameraImageCaptureApp;
	mainApp->Run(argc,argv);
	int x;
	std::cin >> x;
	delete mainApp;
	return 0;
}