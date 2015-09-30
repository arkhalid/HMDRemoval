#pragma once
#include <Windows.h>
#include <NuiApi.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#undef min
#undef max

#include "opencv2/opencv.hpp"
#include<vector>
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudWithNormals;

class KinectCapture
{
public:
	KinectCapture();
	~KinectCapture();
	bool init();
	int GetNumberOfSensors();
	bool acquireSensorDepthData(int sensorIdx);
	bool acquireSensorColorData(int sensorIdx);
	bool assemblePointCloud(int cloudIdx);
	bool GetPointCloud(int sensorIdx, PointCloud& pc);
	bool GetColorImage(int sensorIdx, cv::Mat& img);
	bool GetDepthImage(int sensorIdx, cv::Mat& img);
	
	bool initNuiSensor();
	bool initPCL();
	// Sensor variables
	INuiSensor** m_sensors;
	int m_numSensors;
	HANDLE* m_rgbStream;
	HANDLE* m_depthStream;
	int m_sensorFrameWidth;
	int m_sensorFrameHeight;
	cv::Mat* m_depthImage;
	cv::Mat* m_colorImage;
	BYTE** m_colorTextureData;
	BYTE** m_depthTextureData;
	USHORT** m_rawDepthData;
	PointCloud* m_clouds;
};

