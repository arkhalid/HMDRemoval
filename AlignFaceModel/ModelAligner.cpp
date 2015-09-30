#include "ModelAligner.h"

bool ModelAligner::LoadMatrices(std::string  HMD2CalibPatternPath, std::string CalibPattern2HMDmodelPath, std::string HMDmodel2FaceModel, std::string IRCamera2KinectPath, std::string OculusOrigin2IRCameraPath)
{
	CvMat* temp0, *temp1, *temp2,*temp3,*temp4;
	temp0 = (CvMat*)cvLoad(HMD2CalibPatternPath.c_str(), NULL, NULL, NULL);
	temp1 = (CvMat*)cvLoad(CalibPattern2HMDmodelPath.c_str(), NULL, NULL, NULL);
	temp2 = (CvMat*)cvLoad(HMDmodel2FaceModel.c_str(), NULL, NULL, NULL);
	temp3 = (CvMat*)cvLoad(IRCamera2KinectPath.c_str(), NULL, NULL, NULL);
	temp4 = (CvMat*)cvLoad(OculusOrigin2IRCameraPath.c_str(), NULL, NULL, NULL);

	if (temp0 == NULL || temp1 == NULL || temp2 == NULL ||temp3 == NULL || temp4 == NULL) {
		printf("can't load required transformations, exiting...\n");
		return false;
	}
	else 
	{
		m_HMD2CalibPattern = Mat(temp0);
		m_CalibPattern2HMDmodel = Mat(temp1);
		m_HMDmodel2FaceModel = Mat(temp2);
		m_Kinect2IRCamera = Mat(temp3);
		m_Kinect2IRCamera = m_Kinect2IRCamera.inv();
		m_IRCamera2OculusOrigin = Mat(temp4);
		m_IRCamera2OculusOrigin = m_IRCamera2OculusOrigin.inv();
	}
}

bool ModelAligner::GetTransforms(cv::Matx44f currOculusOrigin2HMD, float mat1x, float mat1y, float mat1z, float mat2x, float mat2y, float mat2z, cv::Matx44f & Kinect2HMDmodel, cv::Matx44f & Kinect2FaceModel)
{
	cv::Mat Kinect2HMDmodelMat;
	cv::Mat Kinect2FaceModelMat;
	cv::Mat HMDmodel2KinectMat;
	cv::Mat RotateBy180(cv::Matx44f::eye());
	RotateBy180.at<float>(0, 0) = -1;
	RotateBy180.at<float>(1, 1) = -1;
	cv::Mat FlipY(cv::Matx44f::eye());
	FlipY.at<float>(1, 1) = -1;
	//

	m_OculusOrigin2HMD = cv::Mat(currOculusOrigin2HMD);
	Kinect2HMDmodelMat = m_Kinect2IRCamera * m_IRCamera2OculusOrigin * m_OculusOrigin2HMD * RotateBy180 *m_HMD2CalibPattern * m_CalibPattern2HMDmodel;
	Kinect2FaceModelMat = Kinect2HMDmodelMat * m_HMDmodel2FaceModel;
	//Kinect2HMDmodelMat = m_Kinect2IRCamera * m_IRCamera2OculusOrigin * m_OculusOrigin2HMD;// *m_HMD2CalibPattern * m_CalibPattern2HMDmodel;
	//HMDmodel2KinectMat = m_Kinect2IRCamera.inv() * m_IRCamera2OculusOrigin.inv() * m_OculusOrigin2HMD.inv();
	//Kinect2HMDmodelMat = m_Kinect2IRCamera.inv() * m_IRCamera2OculusOrigin.inv() * m_OculusOrigin2HMD.inv() * m_HMD2CalibPattern.inv() * m_CalibPattern2HMDmodel;
	//HMDmodel2KinectMat = m_Kinect2IRCamera.inv() * m_IRCamera2OculusOrigin.inv() * m_OculusOrigin2HMD.inv() * m_HMD2CalibPattern.inv() * m_CalibPattern2HMDmodel.inv();
	//Kinect2HMDmodel = cv::Matx44f((float*)Kinect2HMDmodelMat.data);
	//std::cout << m_Kinect2IRCamera.inv() << std::endl;
	//std::cout << m_IRCamera2OculusOrigin.inv() << std::endl;
	//std::cout << m_OculusOrigin2HMD.inv() << std::endl;
	//std::cout << m_HMD2CalibPattern.inv() << std::endl;
	//std::cout << m_CalibPattern2HMDmodel.inv() << std::endl;
	//Kinect2FaceModelMat = Kinect2HMDmodelMat * m_HMDmodel2FaceModel;
	Kinect2FaceModel = cv::Matx44f((float*)Kinect2FaceModelMat.data);
	Kinect2HMDmodel = cv::Matx44f((float*)Kinect2HMDmodelMat.data);
	return true;
}

void ModelAligner::UpdateOculusOrigin2IRCamera(cv::Matx44f transform)
{
	m_IRCamera2OculusOrigin = cv::Mat(transform).inv();
}

bool ModelAligner::SaveOculusOrigin2IRCamera(std::string Path)
{
	cv::Mat tempMat = m_IRCamera2OculusOrigin.inv();
	CvMat* temp = &CvMat(tempMat);
	cvSave(Path.c_str(), temp);
	return true;
}


ModelAligner::ModelAligner()
{
	m_Kinect2IRCamera = Mat::eye(4, 4,CV_32F);
	m_OculusOrigin2HMD = Mat::eye(4, 4, CV_32F);
	m_Kinect2CalibPattern = Mat::eye(4, 4, CV_32F);
	m_HMD2CalibPattern = Mat::eye(4, 4, CV_32F);
	m_HMDmodel2FaceModel = Mat::eye(4, 4, CV_32F);
	m_CalibPattern2HMDmodel = Mat::eye(4, 4, CV_32F);
}


ModelAligner::~ModelAligner()
{
}