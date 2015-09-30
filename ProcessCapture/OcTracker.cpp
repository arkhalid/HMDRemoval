#include "OcTracker.h"
#include "OVR_CAPI.h"
#include <cstdio>
OcTracker::OcTracker():m_haveHMDConnected(false),m_havePositionTracker(false),m_haveVisionTracking(false)
{
	m_hmd = new ovrHmd();
}
OcTracker::~OcTracker()
{
	ovrHmd_Destroy(*m_hmd);
	ovr_Shutdown();
}


bool OcTracker::InitOculus()
{
	ovr_Initialize();
	*m_hmd = ovrHmd_Create(0);
	if (*m_hmd)
	{
		
		ovrHmd_ConfigureTracking(*m_hmd, ovrTrackingCap_Orientation |
			ovrTrackingCap_MagYawCorrection |
			ovrTrackingCap_Position, 0);
		return true;
	}
	else
	{
		return false;
	}
}
bool OcTracker::GetRiftTracking(cv::Matx33f& rotation, cv::Vec3f& position)
{
	ovrTrackingState ss;
	ss = ovrHmd_GetTrackingState(*m_hmd, ovr_GetTimeInSeconds());
	// Report vision tracking
	bool hadVisionTracking = m_haveVisionTracking;
	m_haveVisionTracking = (ss.StatusFlags & ovrStatus_PositionTracked) != 0;
	if (m_haveVisionTracking && !hadVisionTracking) printf("Vision Tracking Acquired \n");
	if (!m_haveVisionTracking && hadVisionTracking) printf("Lost Vision Tracking \n");

	// Report position tracker
	bool hadPositionTracker = m_havePositionTracker;
	m_havePositionTracker = (ss.StatusFlags & ovrStatus_PositionConnected) != 0;
	if (m_havePositionTracker && !hadPositionTracker) printf("Position Tracker Connected \n");
	if (!m_havePositionTracker && hadPositionTracker) printf("Position Tracker Disconnected \n");

	// Report position tracker
	bool hadHMDConnected = m_haveHMDConnected;
	m_haveHMDConnected = (ss.StatusFlags & ovrStatus_HmdConnected) != 0;
	if (m_haveHMDConnected && !hadHMDConnected) printf("HMD Connected \n");
	if (!m_haveHMDConnected && hadHMDConnected) printf("HMD Disconnected \n");
	ss = ovrHmd_GetTrackingState(*m_hmd, ovr_GetTimeInSeconds());
	rotation = OculusQuatTocvMat33f(ss.HeadPose.ThePose.Orientation);
	
	ovrVector3f pos = ss.HeadPose.ThePose.Position;
	position = cv::Vec3f(pos.x, pos.y, pos.z);

	return m_haveVisionTracking && m_havePositionTracker;
}

bool OcTracker::GetRiftTracking(cv::Matx44f& transform)
{
	cv::Matx33f rotation;
	cv::Vec3f position;
	bool state;
	state = this->GetRiftTracking(rotation, position);
	transform = cv::Matx44f::eye();
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			transform(i, j) = rotation(i, j);
		}
	}
	transform(0, 3) = position[0]*100;
	transform(1, 3) = position[1]*100;
	transform(2, 3) = position[2]*100;
	return state;
}
cv::Matx33f OcTracker::OculusQuatTocvMat33f(ovrQuatf orientation)
{
	cv::Matx33f out;
	float qx = orientation.x;
	float qy = orientation.y;
	float qz = orientation.z;
	float qw = orientation.w;
	out(0, 0) = 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2);
	out(0, 1) = 2 * qx*qy - 2 * qz*qw;
	out(0, 2) = 2 * qx*qz + 2 * qy*qw;
	out(1, 0) = 2 * qx*qy + 2 * qz*qw;
	out(1, 1) = 1 - 2 * pow(qx,2) - 2 * pow(qz,2);
	out(1, 2) = 2 * qy*qz - 2 * qx*qw;
	out(2, 0) = 2 * qx*qz - 2 * qy*qw;
	out(2, 1) = 2 * qy*qz + 2 * qx*qw;
	out(2, 2) = 1 - 2 * pow(qx,2) - 2 * pow(qy,2);
	return out;
}

bool OcTracker::GetCameraTracking(cv::Matx44f& transform)
{
	cv::Matx33f rotation;
	cv::Vec3f position;
	ovrTrackingState ss;
	ss = ovrHmd_GetTrackingState(*m_hmd, ovr_GetTimeInSeconds());
	// Report vision tracking
	bool haveCameraTracking = false;
	bool hadVisionTracking = m_haveVisionTracking;
	m_haveVisionTracking = (ss.StatusFlags & ovrStatus_PositionTracked) != 0;
	if (m_haveVisionTracking && !hadVisionTracking) printf("Vision Tracking Acquired \n");
	if (!m_haveVisionTracking && hadVisionTracking) printf("Lost Vision Tracking \n");

	// Report position tracker
	bool hadPositionTracker = m_havePositionTracker;
	m_havePositionTracker = (ss.StatusFlags & ovrStatus_PositionConnected) != 0;
	if (m_havePositionTracker && !hadPositionTracker) printf("Position Tracker Connected \n");
	if (!m_havePositionTracker && hadPositionTracker) printf("Position Tracker Disconnected \n");

	// Report position tracker
	bool hadHMDConnected = m_haveHMDConnected;
	m_haveHMDConnected = (ss.StatusFlags & ovrStatus_HmdConnected) != 0;
	if (m_haveHMDConnected && !hadHMDConnected) printf("HMD Connected \n");
	if (!m_haveHMDConnected && hadHMDConnected) printf("HMD Disconnected \n");
	ss = ovrHmd_GetTrackingState(*m_hmd, ovr_GetTimeInSeconds());
	rotation = OculusQuatTocvMat33f(ss.CameraPose.Orientation);
	ovrVector3f pos = ss.CameraPose.Position;
	 position = cv::Vec3f(pos.x, pos.y, pos.z);
	haveCameraTracking = m_haveHMDConnected = (ss.StatusFlags & ovrStatus_CameraPoseTracked) != 0;
	transform = cv::Matx44f::eye();
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			transform(i, j) = rotation(i, j);
		}
	}
	transform(0, 3) = position[0] * 100;
	transform(1, 3) = position[1] * 100;
	transform(2, 3) = position[2] * 100;
		
	return m_haveVisionTracking && m_havePositionTracker && haveCameraTracking;
}