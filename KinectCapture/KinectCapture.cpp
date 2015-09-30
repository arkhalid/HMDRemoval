#include "KinectCapture.h"
#include <iostream> 


bool KinectCapture::init()
{
	if (!this->initNuiSensor()) {
		std::cout << "The Sensor(s) could not be initialized." << std::endl;
		return false;
	}
	m_colorImage = new cv::Mat[m_numSensors];
	m_depthImage = new cv::Mat[m_numSensors];
	m_colorTextureData = new BYTE*[m_numSensors];
	m_depthTextureData = new BYTE*[m_numSensors];
	m_rawDepthData = new USHORT*[m_numSensors];
	for (size_t i = 0; i < m_numSensors; i++)
	{
		m_colorTextureData[i] = new BYTE[4 * m_sensorFrameHeight * m_sensorFrameWidth];
		m_depthTextureData[i] = new BYTE[m_sensorFrameHeight * m_sensorFrameWidth];
		m_rawDepthData[i] = new USHORT[m_sensorFrameHeight * m_sensorFrameWidth];
	}
	if (!this->initPCL())
	{
		std::cout << "The PointCloud(s) could not be initialized." << std::endl;
		return false;
	}
}

bool KinectCapture::acquireSensorDepthData(int sensorIdx)
{
	HRESULT rs;
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT lockedRect;
	rs = m_sensors[sensorIdx]->NuiImageStreamGetNextFrame(m_depthStream[sensorIdx], 10, &imageFrame);
	if (FAILED(rs)) {
		return false;
	}
	INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
	pTexture->LockRect(0, &lockedRect, NULL, 0);

	if (lockedRect.Pitch != 0)
	{
		const USHORT* pBufferRun = (const USHORT*)(lockedRect.pBits);
		//const BYTE* colorData = _textureData[2 * sensorIdx];
		BYTE* dest = m_depthTextureData[sensorIdx];
		USHORT* destRawDepth = m_rawDepthData[sensorIdx];
		//GLubyte* dest = _textureData[(2 * sensorIdx) + 1];
		ZeroMemory(dest, m_sensorFrameWidth * m_sensorFrameHeight);
		ZeroMemory(destRawDepth, m_sensorFrameWidth * m_sensorFrameHeight);
		//PointCloud* cloudPtr = &m_clouds[sensorIdx];

		//cloudPtr->clear();
		//nt k = 0;

		for (int r = 0; r < m_sensorFrameHeight; ++r)
		{
			for (int c = 0; c < m_sensorFrameWidth; ++c)
			{
				//LONG x, y;
				USHORT depth = NuiDepthPixelToDepth(*(pBufferRun++));
				BYTE* colorDataPtr = m_colorTextureData[sensorIdx];
				//USHORT shiftedDepth = depth << 3;
				*(dest++) = depth % 256;
				*(destRawDepth++) = depth;
				//Vector4 pos = NuiTransformDepthImageToSkeleton(c, r, shiftedDepth);
				//NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
					//NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL,
					//c, r, shiftedDepth, &x, &y);
				//BYTE red, green, blue;
				//if ((x >= 0) && (y >= 0) && (x < m_sensorFrameWidth) && (y < m_sensorFrameHeight)) {
				//	red = colorDataPtr[4 * (r*m_sensorFrameWidth + c) + 0];
				//	green = colorDataPtr[4 * (r*m_sensorFrameWidth + c) + 1];
				//	blue = colorDataPtr[4 * (r*m_sensorFrameWidth + c) + 2];
				//}
				//else {
				//	red = 0;
				//	green = 0;
				//	blue = 0;
				//	k += 3;
				//}

				//
				//pcl::PointXYZRGB pt(red, green, blue);
				//pt.x = pos.x / pos.z;
				//pt.y = pos.y / pos.z;
				//pt.z = pos.z / pos.w;
				//cloudPtr->push_back(pt);
				
			}
		}

		/*
		char pcdFilename[256];
		sprintf_s(pcdFilename, "P:/asad/VersionZero/PointClouds/pointcloud_%d_%.4d.pcd", sensorIdx, _timeStepCount);
		pcl::io::savePCDFileASCII(pcdFilename, *cloudPtr);
		*/
	}
	m_depthImage[sensorIdx] = cv::Mat(m_sensorFrameHeight, m_sensorFrameWidth, CV_8U, m_depthTextureData[sensorIdx]);
	pTexture->UnlockRect(0);
	//pTexture->Release();
	rs = m_sensors[sensorIdx]->NuiImageStreamReleaseFrame(m_depthStream[sensorIdx], &imageFrame);
	return true;
}
bool KinectCapture::acquireSensorColorData(int sensorIdx)
{
	HRESULT rs;
	NUI_IMAGE_FRAME imageFrame;

	rs = m_sensors[sensorIdx]->NuiImageStreamGetNextFrame(m_rgbStream[sensorIdx], 0, &imageFrame);
	if (FAILED(rs)) {
		return false;
	}

	NUI_LOCKED_RECT lockedRect;
	INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
	rs = pTexture->LockRect(0, &lockedRect, NULL, 0);

	if (lockedRect.Pitch != 0) {
		BYTE* bits = static_cast<BYTE*>(lockedRect.pBits);
		BYTE* dest = m_colorTextureData[sensorIdx];
		for (int r = 0; r < m_sensorFrameHeight; ++r) {
			for (int c = 0; c < m_sensorFrameWidth; ++c) {
				dest[0] = bits[0];
				dest[1] = bits[1];
				dest[2] = bits[2];
				dest[3] = 255;
				dest += 4;
				bits += 4;
			}
		}
	}
	m_colorImage[sensorIdx] = cv::Mat(m_sensorFrameHeight, m_sensorFrameWidth, CV_8UC4, m_colorTextureData[sensorIdx]);
	pTexture->UnlockRect(0);
	m_sensors[sensorIdx]->NuiImageStreamReleaseFrame(m_rgbStream[sensorIdx], &imageFrame);
	return true;
}

bool KinectCapture::assemblePointCloud(int sensorIdx)
{
	PointCloud* cloudPtr = &m_clouds[sensorIdx];
	cloudPtr->clear();
	int k = 0;
	LONG x, y;
	USHORT* depthPtr= m_rawDepthData[sensorIdx];
	BYTE* colorDataPtr = m_colorTextureData[sensorIdx];
	for (int r = 0; r < m_sensorFrameHeight; ++r)
	{
		for (int c = 0; c < m_sensorFrameWidth; ++c)
		{
			LONG x, y;
			USHORT shiftedDepth = *depthPtr << 3;
			Vector4 pos = NuiTransformDepthImageToSkeleton(c, r, shiftedDepth);
			NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
				NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL,
				c, r, shiftedDepth, &x, &y);
			BYTE red, green, blue;
			if ((x >= 0) && (y >= 0) && (x < m_sensorFrameWidth) && (y < m_sensorFrameHeight))
			{
				red = colorDataPtr[4*(y*m_sensorFrameWidth + x) + 2];
				green = colorDataPtr[4 * (y*m_sensorFrameWidth + x) + 1];
				blue = colorDataPtr[4 * (y*m_sensorFrameWidth + x) + 0];
			}
			else
			{
				red = 0;
				green = 0;
				blue = 0;
			}
			float realDepth = shiftedDepth / 8000.0;
			if (realDepth < 2)
			{
				pcl::PointXYZRGB pt(red, green, blue);
				pt.x = (c - ((m_sensorFrameWidth/2) -0.5))/( m_sensorFrameWidth) * realDepth;
				pt.y = ((m_sensorFrameHeight - r) - ((m_sensorFrameHeight / 2) - 0.5)) /(m_sensorFrameHeight) * realDepth;;
				pt.z = realDepth;
				cloudPtr->push_back(pt);
			}
			depthPtr++;
			
		}
	}

	return true;
}

bool KinectCapture::GetPointCloud(int sensorIdx, PointCloud& pc)
{
	if (sensorIdx >= m_numSensors)
	{
		return false;
	}
	else
	{
		pc = m_clouds[sensorIdx];
		//std::cout<<pc.points[420].x << "  " << pc.points[420].y << pc.points[420].z << std::endl;
		return true;
	}
}

bool KinectCapture::GetColorImage(int sensorIdx, cv::Mat & img)
{
	if (sensorIdx >= m_numSensors)
	{
		return false;
	}
	else
	{
		img = (m_colorImage[sensorIdx]);
		return true;
	}
}

bool KinectCapture::GetDepthImage(int sensorIdx, cv::Mat & img)
{
	if (sensorIdx >= m_numSensors)
	{
		return false;
	}
	else
	{
		img = (m_depthImage[sensorIdx]);
		return true;
	}
	return false;
}


bool KinectCapture::initNuiSensor()
{
	HRESULT rs, rs1, rs2;
	rs = NuiGetSensorCount(&m_numSensors);
	if (FAILED(rs)) {
		return false;
	}
	if (m_numSensors < 1) {
		return false;
	}

	m_sensors = new INuiSensor*[m_numSensors];
	m_rgbStream = new HANDLE[m_numSensors];
	m_depthStream = new HANDLE[m_numSensors];

	int sensorIdx = 0;
	INuiSensor* sensor;
	for (int i = 0; i < m_numSensors; i++) 
	{
		sensor = NULL;
		rs = NuiCreateSensorByIndex(i, &sensor);
		if (FAILED(rs)) 
		{
			continue;
		}
		rs = sensor->NuiStatus();
		if (rs == S_OK) {
			rs = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
			if (SUCCEEDED(rs)) {
				rs1 = sensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_DEPTH,          // Depth camera or rgb camera?
					NUI_IMAGE_RESOLUTION_640x480,  // Image resolution
					0,                             // Image stream flags, e.g. near mode
					2,                             // Number of frames to buffer
					NULL,                          // Event handle
					&m_depthStream[sensorIdx]);
				rs2 = sensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_COLOR,          // Depth camera or rgb camera?
					NUI_IMAGE_RESOLUTION_640x480,  // Image resolution
					0,                             // Image stream flags, e.g. near mode
					2,                             // Number of frames to buffer
					NULL,                          // Event handle
					&m_rgbStream[sensorIdx]);
				if (SUCCEEDED(rs1) && SUCCEEDED(rs2)) {
					m_sensors[sensorIdx] = sensor;
					sensorIdx++;
					//break;
				}
			}
			else {
				std::cout << "Sensor number " << i << " could not be initialized." << std::endl;
			}
		}
	}
	m_numSensors = sensorIdx;
	std::cout << m_numSensors << " sensors were successfully initialized." << std::endl;

	if ((m_numSensors == 0) || FAILED(rs1) || FAILED(rs2) || FAILED(rs))
	{
		delete[] m_sensors;
		delete[] m_depthStream;
		delete[] m_rgbStream;
		return false;
	}
	m_sensorFrameWidth = 640;
	m_sensorFrameHeight = 480;
	return true;
}

bool KinectCapture::initPCL()
{
	m_clouds = new PointCloud[m_numSensors];
	for (int i = 0; i < m_numSensors; ++i) 
	{
		m_clouds[i].reserve(m_sensorFrameWidth * m_sensorFrameHeight);
	}
	return true;
}

KinectCapture::KinectCapture() :m_depthImage(NULL), m_sensors(NULL), m_rgbStream(NULL), m_depthStream(NULL), m_numSensors(0)
{

}

KinectCapture::~KinectCapture()
{
}

int KinectCapture::GetNumberOfSensors()
{
	return m_numSensors;
}