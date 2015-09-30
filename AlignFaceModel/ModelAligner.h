#pragma once
#include "opencv2/opencv.hpp"
#include <string>

using namespace cv;
class ModelAligner
{
public:
	ModelAligner();
	~ModelAligner();
	bool LoadMatrices(std::string  HMD2CalibPatternPath,std::string CalibPattern2HMDmodelPath,std::string HMDmodel2FaceModel,std::string IRCamera2KinectPath,std::string OculusOrigin2IRCameraPath);
	bool GetTransforms(cv::Matx44f currHMD2OculusOrigin,float mat1x, float mat1y, float mat1z, float mat2x, float mat2y, float mat2z, cv::Matx44f & Kinect2HMDmodel, cv::Matx44f & Kinect2FaceModel);
	void UpdateOculusOrigin2IRCamera(cv::Matx44f OculusOrigin2IRCamera);
	bool SaveOculusOrigin2IRCamera(std::string Path);

	cv::Mat m_Kinect2IRCamera;
	cv::Mat m_OculusOrigin2HMD;
	cv::Mat m_IRCamera2OculusOrigin;
	cv::Mat m_Kinect2CalibPattern;
	cv::Mat m_HMD2CalibPattern;
	cv::Mat m_CalibPattern2HMDmodel;
	cv::Mat m_HMDmodel2FaceModel;
	cv::Mat m_Kinect2FaceModel;
	cv::Mat m_Kinect2HMDmodel;
private:
	// WRLD2OBJ denotes OBJ coordinate system with respect to the WRLD coordinate system

};
