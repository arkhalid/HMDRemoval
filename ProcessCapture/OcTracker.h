#pragma once
#include "OVR_CAPI.h"
#include "opencv2/opencv.hpp"

class OcTracker
{
public:
	OcTracker();
	~OcTracker();
	bool InitOculus();
	bool GetRiftTracking(cv::Matx33f& rotation,cv::Vec3f& position);
	bool GetRiftTracking(cv::Matx44f& transform);
	bool GetCameraTracking(cv::Matx44f& transform);
private:
	ovrHmd* m_hmd;
	bool m_haveVisionTracking;
	bool m_havePositionTracker;
	bool m_haveHMDConnected;
	cv::Matx33f OculusQuatTocvMat33f(ovrQuatf orientation);

};



