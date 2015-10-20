#pragma once
#include "GLHelper.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <GL/glut.h>
using namespace std;
using namespace algebra3;
using namespace cv;
#define PI 3.1415926535f
class OccludedFaceTextureMapper
{
public:
	OccludedFaceTextureMapper();
	~OccludedFaceTextureMapper();
	bool Run(int argc, char* argv[]);
private:
	cv::Mat  cameraPoseEstimate(vector<cv::Point2f> imagePoints, vector < cv::Point3f> worldPoints);
	vector<float> GetUVcoords(Mat K, Mat RT, vector<float> verts, int width, int height);
	bool loadCalibData(string camMatrixFilename, string distMatrixFilename);

	void UpdateTexturesUsingLiveData();

	void display();
	void CheckForSaveImage();
	algebra3::mat4 mat4FromFloatArray(float * in);
	void drawCoordinateAxis(int length);
	float deg2rad(float degrees);
	void keyFunc(unsigned char key, int x, int y);
	bool initCameras();
	bool createRenderObjects();
	bool init (int argc, char** argv);
	bool initGLUT(int argc, char ** argv);
	bool initGL();
	void build_opengl_projection_for_intrinsics(algebra3::mat4 & projection, double alpha, double beta, double skew, double u0, double v0, int img_width, int img_height, double near_clip, double far_clip);
	bool initCameraSim();
	void friend displayGL();
	void friend keyGL(unsigned char key, int x, int y);
	Mat findCameraPoseUsingImage(Mat img);

	struct RenderModelInfo
	{
		RenderObject* mesh;
		algebra3::mat4 modelTranform;
	};
	RenderModelInfo m_faceModelInfo;
	struct CameraInfo
	{
		VideoCapture cap;
		cv::Mat camMat;
		cv::Mat distMat;
		Texture* imageTex;
		int imgWidth;
		int imgHeight;
	};

	mat4 m_projection;
	mat4 m_view;
	GLuint m_programMesh;
	int m_windowWidth;
	int m_windowHeight;
	Mat m_Camera2Face;
	cv::Mat m_camMat;
	cv::Mat m_distMat;
	bool m_untextured_mode;
	bool m_saveImage; //enable saving the rendered scene

	//Live image related variables
	Texture* m_imageTex;
	CameraInfo m_cam1Info;

	//Image test from camera
	bool m_test_mode_image;
	//Texturing test
	RenderModelInfo m_test_TexturingTestModelInfo;
	bool m_test_mode;
	GLuint m_programTexturedMesh;
	
	//View and Object motion Key binding
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
};
