#include "MultiCameraCaptureApp.h"
MultiCameraCaptureApp::MultiCameraCaptureApp() :m_saveImage(false)
{
}

MultiCameraCaptureApp::~MultiCameraCaptureApp()
{
	for each (camInfo cam in m_camList)
	{
		cam.cap.release();
	}
}

bool MultiCameraCaptureApp::Run(int argc, char * argv[])
{
	if (!init())
	{
		std::cout << "Couldn't initialize program" << std::endl;
		return false;
	}
	else
	{
		if (m_camList.size() > 0)
		{
			cv::Mat  images = cv::Mat::zeros(m_camList[0].imgHeight ,m_camList.size() *  m_camList[0].imgWidth,CV_8UC3);
			while (true)
			{
				bool allAcquired = true;
				for each (camInfo cam in m_camList)
				{
					cam.cap >> cam.img;
					if (cam.img.empty())
					{
						allAcquired = false;
					}
					else
					{

					}
				}
				if (allAcquired)
				{
					for (int i = 0; i < m_camList.size();i++)
					{
						camInfo cam;
						cam = m_camList[i];
						cam.img.copyTo(images.colRange(m_camList[0].imgWidth * i, m_camList[0].imgWidth * (i + 1)));
					}
					cv::imshow("Images", images);
					int key = cv::waitKey(30);
					if (key == 's')
					{
						m_saveImage = true;
					}
					if (m_saveImage)
					{
						
						for (int i = 0; i < m_camList.size(); i++)
						{
							stringstream ss;
							ss << i + 1<<"_" <<m_counter << ".png";
							cv::imwrite(ss.str(), m_camList[i].img);
							ss.clear();
						}
						m_counter++;
						m_saveImage = false;
					}
				}
				
			}
		}
	}
	return true;
}

bool MultiCameraCaptureApp::init()
{
	if (!initCams())
	{
		std::cout << "Could not initialize cams" << std::endl;
		return false;
	}
	return true;
}

bool MultiCameraCaptureApp::initCams()
{
	int map[] = { 0,3,1,2 };
	for (int i = 1; i < 10; i++)
	{
		cv::VideoCapture cap;
		cap.open(map[i]);
		if (cap.isOpened())
		{
			camInfo cam;
			cam.cap = cap;
			cam.imgWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
			cam.imgHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
			cam.img = cv::Mat::zeros(cam.imgHeight, cam.imgWidth, CV_8UC3);
			m_camList.push_back(cam);
		}
		else
		{
			break;
		}
	}
	return true;
}

MultiCameraCaptureApp* mainApp;
int main(int argc, char* argv[])
{
	mainApp = new MultiCameraCaptureApp;
	mainApp->Run(argc, argv);
	int x;
	std::cin >> x;
	delete mainApp;
	return 0;
}