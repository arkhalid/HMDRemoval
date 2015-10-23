#include "OccludedFaceTextureMappingApp.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <string>
using namespace std;
OccludedFaceTextureMapper::OccludedFaceTextureMapper() :m_windowWidth(800), m_windowHeight(800), m_test_mode(false), m_test_mode_image(false), m_untextured_mode(false), m_saveImage(false)
{
	m_transx = 0;
	m_transy = 0;
	m_transz = 0;
	m_rotx = 0;
	m_roty = 0;
	m_rotz = 0;
	m_sclx = 1;
	m_scly = 1;
	m_sclz = 1;
	m_transxobj = 0;
	m_transyobj = 0;
	m_transzobj = 0;
	m_rotxobj = 0;
	m_rotyobj = 0;
	m_rotzobj = 0;
	m_viewRadius = 0.13;
	m_viewAngle = 160;
	m_movey = 0.08;
}

OccludedFaceTextureMapper::~OccludedFaceTextureMapper()
{
}

bool OccludedFaceTextureMapper::Run(int argc, char* argv[])
{
	if (!init(argc, argv))
	{
		return false;
		std::cout << "Couldn't initialize program" << std::endl;
	}
	glutMainLoop();
	return true;
}

cv::Mat OccludedFaceTextureMapper::cameraPoseEstimate(vector<cv::Point2f> imagePoints, vector<cv::Point3f> worldPoints)
{
	cv::Mat out = cv::Mat::zeros(4, 4, CV_32F);
	cv::Mat rvec = cv::Mat::zeros(3,1,CV_32F);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
	cv::Mat rotMat = cv::Mat::zeros(3,3,CV_32F);
	m_distMat = cv::Mat::zeros(5, 1, CV_32F);
	cv::solvePnP(worldPoints, imagePoints, m_camMat, m_distMat, rvec, tvec,false, CV_P3P);
	cv::Rodrigues(rvec, rotMat);
	rotMat.copyTo(out.rowRange(0, 3).colRange(0, 3));
	tvec.copyTo(out.rowRange(0, 3).col(3));
	out.at<float>(3, 3) = 1;
	std::cout << out << std::endl;
	return out;
}

vector<float> OccludedFaceTextureMapper::GetUVcoords(Mat K, Mat RT, vector<float> verts, int width, int height)
{
	int numVerts = verts.size() / 3;
	std::cout <<"Vertices "<< numVerts << std::endl;
	vector <float> uvCoords;
	uvCoords.reserve(numVerts*2);
	for (size_t i = 0; i < numVerts; i++)
	{
		Matx41f X;
		X(0, 0) = verts[3 * i];
		X(1, 0) = verts[3 * i + 1];
		X(2, 0) = verts[3 * i + 2];
		X(3, 0) = 1.0;
		Mat x;
		x = K* RT.rowRange(0, 3)*Mat(X);
		x = x / x.at<float>(2, 0);
		uvCoords.push_back(x.at<float>(0, 0) /width);
		uvCoords.push_back(x.at<float>(1, 0) / height);
	}
	return uvCoords;
}

bool OccludedFaceTextureMapper::loadCalibData(string camMatrixFilename, string distMatrixFilename)
{
	CvMat* temp0, *temp1;
	temp0 = (CvMat*)cvLoad(camMatrixFilename.c_str(), NULL, NULL, NULL);
	temp1 = (CvMat*)cvLoad(distMatrixFilename.c_str(), NULL, NULL, NULL);
	if (temp0 == NULL || temp1 == NULL)
	{
		return false;
	}
	else
	{
		m_camMat = cv::Mat(temp0);
		m_distMat = cv::Mat(temp1);
		m_cam1Info.camMat = cv::Mat(temp0);;
		m_cam1Info.distMat = cv::Mat(temp1);;
	}
	return true;
}

void OccludedFaceTextureMapper::UpdateTexturesUsingLiveData()
{
	Mat img;
	m_cam1Info.cap >> img;

	if (img.rows>0 && img.cols>0)
	{
		vector<Mat> imageChannels(3);
		split(img, imageChannels);
		imageChannels.push_back(255 * Mat::ones(img.rows, img.cols, CV_8U));
		merge(imageChannels, img);
		undistort(img, img, m_cam1Info.camMat, m_cam1Info.distMat);
		if (!m_cam1Info.imageTex->UpdateTexture(img))
		{
			std::cout << "Could not update texture" << std::endl;
		}
	}
}
void OccludedFaceTextureMapper::display()
{
	//Update texture using live image
	//UpdateTexturesUsingLiveData();
	
	//Rendering model to scene

	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
	glClearDepth(1.0f);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	//Compute the lookAt matrix using fixed pipeline and reading back (Bad idea ... Need to check LookAT in algebra3)
	/*GLfloat matrix[16];
	glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
	glLoadIdentity();
	gluLookAt(m_viewRadius*sin(deg2rad(m_viewAngle)), m_movey, m_viewRadius*cos(deg2rad(m_viewAngle)), 0, 0, 0, 0, 1, 0);
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	glLoadIdentity();
	m_view = mat4FromFloatArray(matrix);
	m_view = m_view.transpose();
	glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);*/
	m_view = translation3D(vec3(m_transx, m_transy, m_transz)) * rotation3D(vec3(1, 0, 0), m_rotx) * rotation3D(vec3(0, 1, 0), m_roty) * rotation3D(vec3(0, 0, 1), m_rotz) * scaling3D(vec3(m_sclx, m_scly, m_sclz));
	double view[] = { m_view[0][0] ,m_view[1][0],m_view[2][0],m_view[3][0],
		m_view[0][1] ,m_view[1][1],m_view[2][1],m_view[3][1],
		m_view[0][2] ,m_view[1][2],m_view[2][2],m_view[3][2],
		m_view[0][3] ,m_view[1][3],m_view[2][3],m_view[3][3] };
	
	if (!m_test_mode && !m_test_mode_image && !m_untextured_mode)
	{
		//Render the face mesh
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_cam1Info.imageTex->GetTexture());
		RenderObject* currModel = m_faceModelInfo.mesh;
		glUseProgram(m_programTexturedMesh);
		mat4 model_view = m_view * m_faceModelInfo.modelTranform;
		setUniformMat4(m_programTexturedMesh, "modelview", model_view);
		setUniformInt(m_programTexturedMesh, "tex", 0);
		glBindVertexArray(currModel->GetVAO());
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, currModel->GetElemBuff());
		glDrawElements(
			GL_TRIANGLES,      // mode
			currModel->GetIndCount(),   // count
			GL_UNSIGNED_INT,   // type
			(void*)0           // element array buffer offset
			);
		//
		glBindVertexArray(0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glUseProgram(0);
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	else if (!m_test_mode && !m_test_mode_image && m_untextured_mode)
	{
		//Render the face mesh
		RenderObject* currModel = m_faceModelInfo.mesh;
		glUseProgram(m_programMesh);
		mat4 model_view = m_view * m_faceModelInfo.modelTranform;
		setUniformMat4(m_programMesh, "modelview", model_view);
		glBindVertexArray(currModel->GetVAO());
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, currModel->GetElemBuff());
		glDrawElements(
			GL_TRIANGLES,      // mode
			currModel->GetIndCount(),   // count
			GL_UNSIGNED_INT,   // type
			(void*)0           // element array buffer offset
			);
		//
		glBindVertexArray(0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glUseProgram(0);
	}
	else if (!m_test_mode)
	{
		//glViewport(0, 0, 640, 480);
		RenderObject* currModel = m_faceModelInfo.mesh;
		glUseProgram(m_programMesh);
		//mat4 model_view =m_faceModelInfo.modelTranform;
		mat4 model_view = mat4FromFloatArray((float *)(m_Camera2Face.data));
		setUniformMat4(m_programMesh, "modelview", model_view);
		setUniformMat4(m_programMesh, "projection", m_projection);
		glBindVertexArray(currModel->GetVAO());
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, currModel->GetElemBuff());
		glDrawElements(
			GL_TRIANGLES,      // mode
			currModel->GetIndCount(),   // count
			GL_UNSIGNED_INT,   // type
			(void*)0           // element array buffer offset
			);
		//
		glBindVertexArray(0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glUseProgram(0);
	}
	else
	{
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_cam1Info.imageTex->GetTexture());
		RenderObject* currModel = m_test_TexturingTestModelInfo.mesh;
		glUseProgram(m_programTexturedMesh);
		mat4 model_view = m_view * m_test_TexturingTestModelInfo.modelTranform;
		setUniformMat4(m_programTexturedMesh, "modelview", model_view);
		setUniformInt(m_programTexturedMesh,"tex", 0);
		
		glBindVertexArray(currModel->GetVAO());
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, currModel->GetElemBuff());
		glDrawElements(
			GL_TRIANGLES,      // mode
			currModel->GetIndCount(),   // count
			GL_UNSIGNED_INT,   // type
			(void*)0           // element array buffer offset
			);
		//
		glBindVertexArray(0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glUseProgram(0);
		glBindTexture(GL_TEXTURE_2D,0);
	}

	CheckForSaveImage();
	glutPostRedisplay();
	glutSwapBuffers();
}

void OccludedFaceTextureMapper::CheckForSaveImage()
{

	if (m_saveImage)
	{
		GLubyte* imData = new GLubyte[m_windowWidth*m_windowHeight * 4];
		glReadPixels(0, 0, m_windowWidth, m_windowHeight, GL_BGRA, GL_UNSIGNED_BYTE, imData);
		cv::Mat img = cv::Mat(m_windowWidth, m_windowHeight, CV_8UC4, imData);
		flip(img, img, 0);
		imwrite("./../Data/RenderOutput.png", img);
		delete[] imData;
		imData = NULL;
		m_saveImage = false;
	}
}
algebra3::mat4 OccludedFaceTextureMapper::mat4FromFloatArray(float * in)
{
	algebra3::mat4 out(vec4(in[0], in[1], in[2], in[3]),
		vec4(in[4], in[5], in[6], in[7]),
		vec4(in[8], in[9], in[10], in[11]),
		vec4(in[12], in[13], in[14], in[15]));
	return out;
}

inline void OccludedFaceTextureMapper::drawCoordinateAxis(int length)
{
	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(length, 0, 0);
	glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, length, 0);
	glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, length);
	glEnd();
}
inline float OccludedFaceTextureMapper::deg2rad(float degrees)
{
	return ((float)degrees*PI / 180);
}
void OccludedFaceTextureMapper::keyFunc(unsigned char key, int x, int y)
{
	switch (key) {
	case '1': m_transx -= 10; printf("m_transx = %f \n", m_transx); break;
	case '2': m_transx += 10; printf("m_transx = %f \n", m_transx);break;
	case '3': m_transy -= 10; printf("m_transy = %f \n", m_transy);  break;
	case '4': m_transy += 10; printf("m_transy = %f \n", m_transy);  break;
	case '5': m_transz -= 10; printf("m_transz = %f \n", m_transz); break;
	case '6': m_transz += 10; printf("m_transz = %f \n", m_transz); break;
	case 'q': m_rotx -= 0.1; printf("m_rotx = %f \n", m_rotx); break;
	case 'w': m_rotx += 0.1; printf("m_rotx = %f \n", m_rotx); break;
	case 'e': m_roty -= 1.0f; printf("m_roty = %f \n", m_roty); break;
	case 'r': m_roty += 1.0f; printf("m_roty = %f \n", m_roty); break;
	case 't': m_rotz -= 0.1; printf("m_rotz = %f \n", m_rotz); break;
	case 'y': m_rotz += 0.1; printf("m_rotz = %f \n", m_rotz); break;
		case 'z': m_sclx /= 1.01; printf("m_sclx = %f \n", m_sclx); break;
		case 'x': m_sclx *= 1.01; printf("m_sclx = %f \n", m_sclx); break;
		case 'c': m_scly /= 1.01; printf("m_scly = %f \n", m_scly); break;
		case 'v': m_scly *= 1.01; printf("m_scly = %f \n", m_scly); break;
		case 'b': m_sclz /= 1.01; printf("m_sclz = %f \n", m_sclz); break;
		case 'n': m_sclz *= 1.01; printf("m_sclz = %f \n", m_sclz); break;
	/*case 'a': m_transxobj -= 1; printf("m_transxobj = %f \n", m_transxobj); break;
	case 's': m_transxobj += 1; printf("m_transxobj = %f \n", m_transxobj);  break;
	case 'd': m_transyobj -= 1; printf("m_transyobj = %f \n", m_transyobj); break;
	case 'f': m_transyobj += 1; printf("m_transyobj = %f \n", m_transyobj);break;
	case 'g': m_transzobj -= 1; printf("m_transzobj = %f \n", m_transzobj);  break;
	case 'h': m_transzobj += 1; printf("m_transzobj = %f \n", m_transzobj); break;*/
		//case 'z': m_rotxobj -= 0.1; printf("m_rotx = %f \n", m_rotxobj); break;
		//case 'x': m_rotxobj += 0.1; printf("m_rotx = %f \n", m_rotxobj); break;
		//case 'c': m_rotyobj -= 0.1f; printf("m_roty = %f \n", m_rotyobj); break;
		//case 'v': m_rotyobj += 0.1f; printf("m_roty = %f \n", m_rotyobj); break;
		//case 'b': m_rotzobj -= 0.1; printf("m_rotz = %f \n", m_rotzobj); break;
		//case 'n': m_rotzobj += 0.1; printf("m_rotz = %f \n", m_rotzobj); break;
/*	case 'z': m_movey = (m_movey > 0.01) ? 0.9f*m_movey : m_movey; printf("m_movey = %f \n", m_movey); break;
	case 'x': m_movey = 1.1*m_movey; printf("m_movey = %f \n", m_movey); break;
	case 'c': m_viewRadius = (m_viewRadius > 0.01) ? 0.9f*m_viewRadius : m_viewRadius; printf("m_viewRadius = %f \n", m_viewRadius); break;
	case 'v': m_viewRadius = 1.1*m_viewRadius; printf("m_viewRadius = %f \n", m_viewRadius); break;
	case 'b': m_viewAngle += 10; printf("m_viewAngle = %f \n", m_viewAngle); break;
	case 'n': m_viewAngle -= 10; printf("m_viewAngle = %f \n", m_viewAngle); break;*/
	case 's':m_saveImage = true; break;
	case 'm': break;
	default: printf("Invalid key for static room rendering \n"); break;
	}
}

bool OccludedFaceTextureMapper::initCameras()
{
	int idx = 1;
	m_cam1Info.cap.open(idx);
	if (!m_cam1Info.cap.isOpened())
	{
		std::cout << "Could not open camera stream for camera " << idx << std::endl;
		return false;
	}
	else
	{
		m_cam1Info.imgWidth = m_cam1Info.cap.get(CV_CAP_PROP_FRAME_WIDTH);
		m_cam1Info.imgHeight = m_cam1Info.cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		return true;
	}

	int camNums[] = { 2,1,3 };

}

bool OccludedFaceTextureMapper::createRenderObjects()
{
	vector<float> vertices;
	vector<float> colors;
	vector<unsigned int> indices;
	vector<float> texCoords;
	ReadPLY("./../Models/Mannequin0CleanCentered.ply", vertices, colors, indices);
	m_faceModelInfo.mesh = new RenderObject(&vertices, &colors, &indices);
	//m_faceModelInfo.modelTranform = identity3D();
	m_faceModelInfo.modelTranform = translation3D(vec3(0, -30, -620)) * rotation3D(vec3(1, 0, 0), -79) * rotation3D(vec3(0, 1, 0), 0) * rotation3D(vec3(0, 0, 1), -145.6);
	vector<cv::Point2f> imgPoints;
	vector<cv::Point3f> worldPoints;

	imgPoints.push_back(cv::Point2f(423.8102 , 381.2220));
	imgPoints.push_back(cv::Point2f(514.0216 ,  82.4928));
	imgPoints.push_back(cv::Point2f(240.5784 , 313.1655));
	imgPoints.push_back(cv::Point2f(282.8874 , 103.0831));


	worldPoints.push_back(cv::Point3f(50.6372  ,122.4560 ,  64.4402));//magenta
	worldPoints.push_back(cv::Point3f(53.7263  ,110.8696 ,  41.4052));//yellow
	worldPoints.push_back(cv::Point3f(36.7116  ,124.9903  , 58.4990));//green
	worldPoints.push_back(cv::Point3f(37.4948 , 120.7907  , 40.8366));//blue
	cv::Mat img;
	img = imread("./../Data/image_new.png");
	m_Camera2Face = cameraPoseEstimate(imgPoints, worldPoints);
	m_cam1Info.Camera2Face = cameraPoseEstimate(imgPoints, worldPoints);
	vector<Mat> image1Channels(3);
	split(img, image1Channels);
	image1Channels.push_back(255 * Mat::ones(img.rows, img.cols, CV_8U));
	merge(image1Channels, img);
	//flip(img, img, 0);
	m_cam1Info.imageTex = new Texture(img, GL_LINEAR, GL_CLAMP_TO_EDGE);
	texCoords = GetUVcoords(m_camMat, m_Camera2Face, vertices, img.cols, img.rows);
	if (!m_faceModelInfo.mesh->AddTexCoords(&texCoords))
	{
		std::cout << "Couldn't add tex coordinates" << std::endl;
	}

	//Calculate reporojection error
	Matx41f pt;
	
	pt = cv::Matx41f(worldPoints[0].x, worldPoints[0].y, worldPoints[0].z, 1);
	Mat proj = m_cam1Info.camMat *  m_Camera2Face.rowRange(0, 3) * Mat(pt);
	proj = proj / proj.at<float>(2, 0);
	std::cout << "Proj 0 = " << proj << std::endl;
	pt = cv::Matx41f(worldPoints[1].x, worldPoints[1].y, worldPoints[1].z, 1);
	proj = m_cam1Info.camMat *  m_Camera2Face.rowRange(0, 3) * Mat(pt);
	proj = proj / proj.at<float>(2, 0);
	std::cout << "Proj 1 = " << proj << std::endl;
	pt = cv::Matx41f(worldPoints[2].x, worldPoints[2].y, worldPoints[2].z, 1);
	proj = m_cam1Info.camMat *  m_Camera2Face.rowRange(0, 3) * Mat(pt);
	proj = proj / proj.at<float>(2, 0);
	std::cout << "Proj 2 = " << proj << std::endl;
	pt = cv::Matx41f(worldPoints[3].x, worldPoints[3].y, worldPoints[3].z, 1);
	proj = m_cam1Info.camMat *  m_Camera2Face.rowRange(0, 3) * Mat(pt);
	proj = proj / proj.at<float>(2, 0);
	std::cout << "Proj 3 = " << proj << std::endl;

	if (m_test_mode)
	{
		float test_verts[] = { -1,-1,-1,-1,1,-1,1,1,-1,1,-1,-1 };
		float test_cols[] = { 1.0,0,0,0,1.0,0,0,0,1,1,1,0 };
		float test_uv[] = { 0,0,0,1,1,1,1,0 };
		unsigned int test_inds[] = { 0,1,2,0,2,3 };
		vector<float> test_vertices(test_verts, test_verts + sizeof(test_verts) / sizeof(test_verts[0]));
		vector<float> test_colors(test_cols, test_cols + sizeof(test_cols) / sizeof(test_cols[0]));
		vector<unsigned int> test_indices(test_inds, test_inds + sizeof(test_inds) / sizeof(test_inds[0]));
		vector<float> test_texCoords(test_uv, test_uv + sizeof(test_uv) / sizeof(test_uv[0]));

		m_test_TexturingTestModelInfo.mesh = new RenderObject(&test_vertices, &test_colors, &test_indices);
		m_test_TexturingTestModelInfo.modelTranform = identity3D();
		if (!m_test_TexturingTestModelInfo.mesh->AddTexCoords(&test_texCoords))
		{
			std::cout << "Couldn't add tex coordinates" << std::endl;
		}
		cv::Mat img;
		img = imread("./../Data/image_0.png");
		vector<Mat> image1Channels(3);
		split(img, image1Channels);
		image1Channels.push_back(255 * Mat::ones(img.rows, img.cols, CV_8U));
		merge(image1Channels, img);
		flip(img, img, 0);
		m_cam1Info.imageTex = new Texture (img, GL_LINEAR, GL_CLAMP_TO_EDGE);
	}
	return true;
}

bool OccludedFaceTextureMapper::init(int argc, char** argv)
{
	if(!initGLUT(argc,argv))
	{
		std::cout << "Couldn't Initialize GLUT" << std::endl;
		return false;
	}
	if (!initGL())
	{
		std::cout << "Couldn't Initialize GL" << std::endl;
		return false;
	}
	if (!loadCalibData("../Data/camCalib/cam_rgb_1.xml", "../Data/camCalib/distort_rgb_1.xml"))
	{
		std::cout << "Could not load calibrations" << std::endl;
		return false;
	}


	if (!initCameras())
	{
		std::cout << "Couldn't initialize Cameras" << std::endl;
		return false;
	}
	if (!createRenderObjects())
	{
		return false;
	}
	if (m_test_mode_image)
	{
		if (!initCameraSim())
		{
			std::cout << "Couldn't initialize cam Sim" << std::endl;
		}
	}


	return true;
}
bool OccludedFaceTextureMapper::initGLUT(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(m_windowWidth, m_windowHeight);
	glutCreateWindow("HMD Telepresence");
	glutDisplayFunc(displayGL);
	glutKeyboardFunc(keyGL);
	return true;
}


bool OccludedFaceTextureMapper::initGL()
{

	int err = glewInit();
	if ((err != 0))
	{
		cout << "GLEW could not be initialized!" << endl;
		cout << "Error code is: " << err << std::endl;
		return false;
	}

	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_MULTISAMPLE_ARB);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glClearColor(0, 0, 0, 1); // Black*/
	m_projection = getPerspectiveMatrix(-0.1, 0.1, -0.1, 0.1, 0.4, 1000);
	m_view = identity3D();



	if (m_test_mode_image)
	{
		m_programMesh = createShaderProgram(
			"./../Shaders/point_cloud.vert",
			"./../Shaders/point_cloud.frag");
		setUniformMat4(m_programMesh, "projection", m_projection);
		setUniformMat4(m_programMesh, "modelview", m_view);
	}
	else
	{
		if (m_untextured_mode)
		{
			m_programMesh = createShaderProgram(
				"./../Shaders/point_cloud.vert",
				"./../Shaders/point_cloud.frag");
			setUniformMat4(m_programMesh, "projection", m_projection);
			setUniformMat4(m_programMesh, "modelview", m_view);
		}
		else
		{
			m_programTexturedMesh = createShaderProgram(
				"./../Shaders/textured_model.vert",
				"./../Shaders/textured_model_experimental2D.frag");
			setUniformMat4(m_programTexturedMesh, "projection", m_projection);
			setUniformMat4(m_programTexturedMesh, "modelview", m_view);
		}
	}
	glUseProgram(0);
	return true;
}

void OccludedFaceTextureMapper::build_opengl_projection_for_intrinsics( algebra3::mat4 &projection,  double alpha, double beta, double skew, double u0, double v0, int img_width, int img_height, double near_clip, double far_clip )
{
     
    // These parameters define the final viewport that is rendered into by
    // the camera.
    double L = 0;
    double R = img_width;
    double B = 0;
    double T = img_height;
     
    // near and far clipping planes, these only matter for the mapping from
    // world-space z-coordinate into the depth coordinate for OpenGL
    double N = near_clip;
    double F = far_clip;
     
    //// set the viewport parameters
    //viewport[0] = L;
    //viewport[1] = B;
    //viewport[2] = R-L;
    //viewport[3] = T-B;
    // 
    // construct an orthographic matrix which maps from projected
    // coordinates to normalized device coordinates in the range
    // [-1, 1].  OpenGL then maps coordinates in NDC to the current
    // viewport
    
	cv::Matx44f ortho = cv::Matx44f::zeros();
    ortho(0,0) =  2.0/(R-L); ortho(0,3) = -(R+L)/(R-L);
    ortho(1,1) =  2.0/(T-B); ortho(1,3) = -(T+B)/(T-B);
    ortho(2,2) = -2.0/(F-N); ortho(2,3) = -(F+N)/(F-N);
    ortho(3,3) =  1.0;
     
    // construct a projection matrix, this is identical to the 
    // projection matrix computed for the intrinsicx, except an
    // additional row is inserted to map the z-coordinate to
    // OpenGL. 
	cv::Matx44f tproj = cv::Matx44f::zeros();
    tproj(0,0) = alpha; tproj(0,1) = skew; tproj(0,2) = u0;
                        tproj(1,1) = beta; tproj(1,2) = v0;
                                           tproj(2,2) = -(N+F); tproj(2,3) = -N*F;
                                           tproj(3,2) = 1.0;
     
    // resulting OpenGL frustum is the product of the orthographic
    // mapping to normalized device coordinates and the augmented
    // camera intrinsic matrix
	cv::Mat out = Mat(ortho)* Mat(tproj);
	std::cout << out << std::endl;
    projection = mat4FromFloatArray((float*)out.data);
}
bool OccludedFaceTextureMapper::initCameraSim()
{
	m_faceModelInfo.modelTranform = mat4FromFloatArray((float*)m_Camera2Face.data);
	float fx = m_camMat.at<float>(0, 0);
	float fy = m_camMat.at<float>(1,1);
	float cx = m_camMat.at<float>(0, 2);
	float cy = m_camMat.at<float>(1, 2);
	float f = 1000;
	float n = 0.1;

	 build_opengl_projection_for_intrinsics(m_projection,fx, fy, 0, cx, cy, 640, 480,0.3,10);
	
	return true;
}
bool detectBlobInImage(Mat img, Vec3d minVal, Vec3d maxVal,Point2f& out )
{
	vector<Mat> components;
	split(img, components);
	//imshow("A", components[0]);
	//imshow("B", components[1]);
	//imshow("C", components[2]);
	waitKey(1);

	Mat thresholdY;
	Mat thresholdCr;
	Mat thresholdCb;
	RotatedRect rect;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	inRange(components[0], Scalar(minVal[0]), Scalar(maxVal[0]), thresholdY);
	//imshow("thresholdY", thresholdY);
	inRange(components[1], Scalar(minVal[1]), Scalar(maxVal[1]), thresholdCr);
	//imshow("thresholdCr", thresholdCr);
	inRange(components[2], Scalar(minVal[2]), Scalar(maxVal[2]), thresholdCb);
	//imshow("thresholdCb", thresholdCb);
	Mat thresholded = thresholdY & thresholdCb & thresholdCr;
	//imshow("Thresholded", thresholded);

	findContours(thresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	if (contours.size() == 0)
	{
		return false;
	}
	else
	{
		int idx = 0, largestComp = 0;
		double maxArea = 0;
		for (; idx >= 0; idx = hierarchy[idx][0])
		{
			const vector<Point>& c = contours[idx];
			double area = fabs(contourArea(Mat(c)));
			if (area > maxArea)
			{
				maxArea = area;
				largestComp = idx;
			}
		}
		Scalar color(0, 0, 255);
		drawContours(img, contours, largestComp, color, CV_FILLED, 8, hierarchy);
		rect = fitEllipse(contours[largestComp]);
		//circle(img, rect.center, 3, Scalar(255, 0, 0));
		out = rect.center;
		imshow("WithContours", img);
	}
	return true;
}

Mat OccludedFaceTextureMapper::findCameraPoseUsingImage(Mat img)
{
	vector<cv::Point2f> imgPoints;
	vector<cv::Point3f> worldPoints;
	/*imgPoints.push_back(cv::Point2f(332.8021047227926, 100.2138090349076));
	imgPoints.push_back(cv::Point2f(576.555427914991, 364.761439785564));
	imgPoints.push_back(cv::Point2f(378.142144638404, 375.658354114713));
	imgPoints.push_back(cv::Point2f(496.547176751885, 151.710134265220));*/
	/*worldPoints.push_back(cv::Point3f(0.218356700000000, 0.0445314000000000, -0.570620883333333));
	worldPoints.push_back(cv::Point3f(0.231745464285714, 0.0259706071428572, -0.575379797619048));
	worldPoints.push_back(cv::Point3f(0.219618000000000, 0.0238599166666667, -0.573512833333333));
	worldPoints.push_back(cv::Point3f(0.229610375000000, 0.0390814583333333, -0.574926486111111));*/
	cv::Point2f blue;
	cv::Point2f yellow;
	cv::Point2f green;
	cv::Point2f magenta;
	Mat imgCrCb;
	cvtColor(img, imgCrCb, CV_BGR2YCrCb);
	if (detectBlobInImage(imgCrCb, Vec3d(0.1 * 255, 115.85, 140.25), Vec3d(0.9 * 255, 125.97, 150.96), blue));
	{
		std::cout << blue.x << "  " << blue.y << std::endl;
	}

	if (detectBlobInImage(imgCrCb, Vec3d(0.1 * 255, 0.363 * 255, 0.464 * 255), Vec3d(0.9 * 255, 0.405 * 255, 0.508 * 255), green))
	{
		std::cout << green.x << "  " << green.y << std::endl;
	}

	if (detectBlobInImage(imgCrCb, Vec3d(0.1 * 255, 0.620 * 255, 0.555 * 255), Vec3d(0.9 * 255, 0.692 * 255, 0.587 * 255), magenta))
	{
		std::cout << magenta.x << "  " << magenta.y << std::endl;
	}

	if (detectBlobInImage(imgCrCb, Vec3d(0.1 * 255,0.474 * 255,  87.21), Vec3d(0.95 * 255, 0.512*255, 106.845), yellow))
	{
		std::cout << yellow.x << "  " << yellow.y << std::endl;
	}




	imgPoints.push_back(blue);
	imgPoints.push_back(green);
	imgPoints.push_back(magenta);
	imgPoints.push_back(yellow);
	worldPoints.push_back(cv::Point3f(0.218356700000000, 0.0445314000000000, -0.570620883333333));
	worldPoints.push_back(cv::Point3f(0.231745464285714, 0.0259706071428572, -0.575379797619048));
	worldPoints.push_back(cv::Point3f(0.219618000000000, 0.0238599166666667, -0.573512833333333));
	worldPoints.push_back(cv::Point3f(0.229610375000000, 0.0390814583333333, -0.574926486111111)); 
	return cameraPoseEstimate(imgPoints, worldPoints);
}


OccludedFaceTextureMapper* mainApp;

void displayGL()
{
	mainApp->display();
}
void keyGL(unsigned char key, int x, int y)
{
	mainApp->keyFunc(key, x, y);
}

int main(int argc, char* argv[])
{
	mainApp = new OccludedFaceTextureMapper;
	mainApp->Run(argc,argv);
	delete mainApp;
	int x;
	cin >> x;
	return 0;
}