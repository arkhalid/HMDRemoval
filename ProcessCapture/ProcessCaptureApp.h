#pragma once
//GL helper has been changed ... Model is no longer scaled by 100 ... This code will no longer work ... Add scaling to this code
#include "GLHelper.h"
#include <iostream>
#include <string.h>
#include "OcTracker.h"
#include "KinectCapture.h"
#include "ModelAligner.h"
#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <GL/glut.h>
#undef min
#undef max
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
#define PI 3.1415926535f
class ProcessCaptureApp
{
public:
	ProcessCaptureApp();
	~ProcessCaptureApp();
	bool Run(int argc, char* argv[]);
	struct RenderModelInfo
	{
		RenderObject* mesh;
		algebra3::mat4 modelTranform;
	};
private:
	bool init(int argc, char** argv);
	bool initGL();
	bool initGLUT(int argc, char** argv);
	bool initKinect();
	bool initOculus();
	bool initFaceAlignment(bool SaveCameraPosition);
	void display();
	void keyFunc(unsigned char key, int x, int y);
	bool createRenderObjects();
	void friend displayGL();
	void friend keyGL(unsigned char key, int x, int y);
	std::vector<float> UnsignedCharToFloat(std::vector<unsigned char>);
	algebra3::mat4 mat4FromFloatArray(float * in);
	void visualizeMatrix(algebra3::mat4 matrix, float length);
	void drawCoordinateAxis(int length);
	void UpdateTrackingData();
	float deg2rad(float degrees);
	void checkForSaveOculusCameraTransform();
	void ModifyMatrix(float a_x, float a_y, float a_z, cv::Mat &InOut);

	algebra3::mat4 m_view;
	int m_WindowWidth;
	int m_WindowHeight;
	GLuint m_programPointCloud;
	vector<RenderModelInfo> m_ModelInfoList;
	vector<PointCloud> m_PointCloudList;
	algebra3::mat4 m_modelview;
	algebra3::mat4 m_projection;
	bool m_special;
	cv::Matx44f m_latestOcTranform;
	bool m_latestOcTrackingState;
	int m_count;
	bool m_VisualizeTransformations;
	//Pointers to other classes
	KinectCapture* m_kinCap;
	OcTracker* m_ocTrack;
	ModelAligner* m_modelAligner;
	
	//key variables
	float m_transx;
	float m_transy;
	float m_transz;
	float m_rotx;
	float m_roty;
	float m_rotz;
	float m_sclx;
	float m_scly;
	float m_sclz;

	float m_transxobj;
	float m_transyobj;
	float m_transzobj;
	float m_rotxobj;
	float m_rotyobj;
	float m_rotzobj;

	float m_viewRadius;
	float m_viewAngle;
	float m_movey;
	bool m_saveCameraPose;

};
