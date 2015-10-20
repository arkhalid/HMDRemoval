#include "ProcessCaptureApp.h"
using namespace algebra3;

bool ProcessCaptureApp::initGLUT(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(m_WindowWidth, m_WindowHeight);
	glutCreateWindow("HMD Telepresence");
	glutDisplayFunc(displayGL);
	glutKeyboardFunc(keyGL);
	return true;
}

std::vector<float> ProcessCaptureApp::UnsignedCharToFloat(std::vector<unsigned char> in)
{
	std::vector <float> out;
	out.resize(in.size());
	for (size_t i = 0; i < in.size(); i++)
	{
		out[i] = static_cast<float>(in[i]);
	}
	return out;
}

algebra3::mat4 ProcessCaptureApp::mat4FromFloatArray(float * in)
{
	algebra3::mat4 out(vec4(in[0],in[1],in[2],in[3]),
		vec4(in[4], in[5], in[6], in[7]),
		vec4(in[8], in[9], in[10], in[11]),
		vec4(in[12], in[13], in[14], in[15]));
	return out;
}

void ProcessCaptureApp::visualizeMatrix(algebra3::mat4 matrix,float length)
{
	double view[] = {m_view[0][0] ,m_view[1][0],m_view[2][0],m_view[3][0],
		m_view[0][1] ,m_view[1][1],m_view[2][1],m_view[3][1],
		m_view[0][2] ,m_view[1][2],m_view[2][2],m_view[3][2],
		m_view[0][3] ,m_view[1][3],m_view[2][3],m_view[3][3]};
	double mat[] = { matrix[0][0] ,matrix[1][0],matrix[2][0],matrix[3][0],
		matrix[0][1] ,matrix[1][1],matrix[2][1],matrix[3][1],
		matrix[0][2] ,matrix[1][2],matrix[2][2],matrix[3][2],
		matrix[0][3] ,matrix[1][3],matrix[2][3],matrix[3][3] };
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixd(view);
	glMultMatrixd(mat);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(1.0, 0.0, 0.0);
	glEnd();
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 1.0, 0.0);
	glEnd();
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0,0.0);
	glVertex3f(0.0, 0.0, 1.0);
	glEnd();
	glLoadIdentity();
}

void ProcessCaptureApp::drawCoordinateAxis(int length)
{
	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(length, 0, 0);
	glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, length, 0);
	glColor3f(1, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0, length);
	glEnd();
}
void ProcessCaptureApp::UpdateTrackingData()
{
		for (int i = 0; i < m_kinCap->GetNumberOfSensors(); i++)
	{
		if (m_kinCap->acquireSensorColorData(i) && m_kinCap->acquireSensorDepthData(i))
		{
			m_kinCap->assemblePointCloud(i);
			m_kinCap->GetPointCloud(i, m_PointCloudList[i]);
		}
	}
	m_latestOcTrackingState = m_ocTrack->GetRiftTracking(m_latestOcTranform);
}
float ProcessCaptureApp::deg2rad(float degrees)
{
		return ((float)degrees*PI / 180);
}
bool ProcessCaptureApp::initKinect()
{
	m_kinCap = new KinectCapture();
	if (!m_kinCap->init())
		{
			std::cout << "Couldn't intialize Kinect" << std::endl;
			return false;
		}
	else
	{
		bool res = true;
		int numSensors = m_kinCap->GetNumberOfSensors();
		if (numSensors > 0)
		{
			for (int i = 0; i < numSensors; i++)
			{
				PointCloud pc;
				bool initPC = false;
				int tries = 0;
				while (!initPC && tries < 100000)
				{
					if (m_kinCap->acquireSensorColorData(i) && m_kinCap->acquireSensorDepthData(i))
					{
							m_kinCap->assemblePointCloud(i);
							m_kinCap->GetPointCloud(i, pc);
							m_PointCloudList.push_back(pc);
							//vector<float> points(&pc.points[0].x, &pc.points[0].x + pc.size());
							//vector<unsigned char> colors(&pc.points[0].r, &pc.points[0].r + pc.size());
							initPC = true;
					}
					tries++;
				}
				if (tries > 100000)
				{
					res = false;
				}
			}
		}
		else
		{
			return false;
		}
		return res;
	}
}

bool ProcessCaptureApp::initOculus()
{
	m_ocTrack = new OcTracker();
	if (!m_ocTrack->InitOculus())
	{
		std::cout << "Couldn't initialize Tracker" << std::endl;
		return false;
	}	else
	{
		return true;
	}
}

bool ProcessCaptureApp::initFaceAlignment(bool SaveCameraPosition)
{
	m_modelAligner = new ModelAligner();
	std::cout << "Initilizing Model Aligner" << std::endl;
	m_modelAligner->LoadMatrices("../Data/HM2calib.xml", "../Data/calib2hmdModelCorrected.xml", "../Data/hmdModel2faceModel.xml", "../Data/rift2kinect.xml", "../Data/OO2IR.xml");
	if (SaveCameraPosition)
	{
		std::cout << "Please bring the Oculus in the tracker's field of view" << std::endl;
		bool found = false;
		cv::Matx44f transform;
		while (!found)
		{
			found = m_ocTrack->GetCameraTracking(transform);
		}
		m_modelAligner->UpdateOculusOrigin2IRCamera(transform);
		m_modelAligner->SaveOculusOrigin2IRCamera("../Data/OO2IR.xml");
		std::cout << cv::Mat(transform) << std::endl;
		std::cout << "Saved Oculus Camera Position" << std::endl;
	}
	return true;
}

float deg2rad(float in) {
	return(in*3.14f / 180.0f);
}

void ProcessCaptureApp::checkForSaveOculusCameraTransform()
{
	if (m_saveCameraPose)
	{
		bool found = false;
		cv::Matx44f transform;
		while (!found)
		{
			std::cout << "RIFT" << std::endl;
			found = m_ocTrack->GetRiftTracking(transform);
			std::cout << "Camera" << std::endl;
			found = m_ocTrack->GetCameraTracking(transform);

		}
		m_modelAligner->UpdateOculusOrigin2IRCamera(transform);
		m_modelAligner->SaveOculusOrigin2IRCamera("../Data/OO2IR.xml");
		std::cout << "Saved Oculus Camera Position" << std::endl;
		m_saveCameraPose = false;
	}
}
void ProcessCaptureApp::display()
{
	UpdateTrackingData();
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
	glClearDepth(1.0f);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	//Creating view Matrix
	//m_view = translation3D(vec3(m_transx, m_transy, m_transz)) * rotation3D(vec3(1, 0, 0), m_rotx) * rotation3D(vec3(0, 1, 0), m_roty) * rotation3D(vec3(0, 0, 1), m_rotz) * scaling3D(vec3(m_sclx, m_scly, m_sclz));
	GLfloat matrix[16];
	glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
	glLoadIdentity();
	gluLookAt(m_viewRadius*sin(deg2rad(m_viewAngle)), m_movey, m_viewRadius*cos(deg2rad(m_viewAngle)), 0, 0, 0, 0, 1, 0);
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	glLoadIdentity();
	m_view = mat4FromFloatArray(matrix);
	m_view = m_view.transpose();
	glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);
	double view[] = { m_view[0][0] ,m_view[1][0],m_view[2][0],m_view[3][0],
		m_view[0][1] ,m_view[1][1],m_view[2][1],m_view[3][1],
		m_view[0][2] ,m_view[1][2],m_view[2][2],m_view[3][2],
		m_view[0][3] ,m_view[1][3],m_view[2][3],m_view[3][3] };

	if (m_VisualizeTransformations)
	{
		glViewport(0, 0, m_WindowWidth, m_WindowHeight);
		glMatrixMode(GL_PROJECTION); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
		glLoadIdentity();
		glFrustum(-0.1, 0.1, -0.1, 0.1, 0.1, 1000);

		glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
		glLoadIdentity();
		glMultMatrixd(view);
		//gluLookAt(m_viewRadius*sin(deg2rad(m_viewAngle)), m_movey, m_viewRadius*cos(deg2rad(m_viewAngle)), 0,0,0, 0, 1, 0);
		glLineWidth(2);
		drawCoordinateAxis(100);

		glPointSize(3);

		for (int i = 0; i < m_PointCloudList.size(); i++)
		{
			for (int j = 0; j < m_PointCloudList[i].size(); j++)
			{
				glBegin(GL_POINTS);
				glColor3f(m_PointCloudList[i].points[j].r / 255.0, m_PointCloudList[i].points[j].g / 255.0, m_PointCloudList[i].points[j].b / 255.0);
				glVertex3f(m_PointCloudList[i].points[j].x * 100, m_PointCloudList[i].points[j].y * 100, m_PointCloudList[i].points[j].z * 100);
				glEnd();
			}
		}

		cv::Mat temp; 
		cv::Mat RotateBy180(cv::Matx44f::eye());
		RotateBy180.at<float>(0, 0) = -1;
		RotateBy180.at<float>(1, 1) = -1;
		cv::Mat FlipY(cv::Matx44f::eye());
		FlipY.at<float>(1, 1) = -1;
		cv::Mat TranslateByKeyValues(cv::Matx44f::eye());
		TranslateByKeyValues.at<float>(0, 3) = m_transxobj;
		TranslateByKeyValues.at<float>(1, 3) = m_transyobj;
		TranslateByKeyValues.at<float>(2, 3) = m_transzobj;

		if (m_latestOcTrackingState)
		{
				cv::Mat Kin2IR = m_modelAligner->m_Kinect2IRCamera;
				//cv::Mat Kin2IRFlip =  Kin2IR;
				temp = Kin2IR.t();
				glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
				glMultMatrixf((GLfloat*)temp.data);
				drawCoordinateAxis(80);
				glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);

				glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
				cv::Mat IR2OO = m_modelAligner->m_IRCamera2OculusOrigin;
				cv::Mat Kin2OO = Kin2IR * IR2OO;
				temp = Kin2OO.t();
				glMultMatrixf((GLfloat*)temp.data);
				//drawCoordinateAxis(1);
				glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);
				
				glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
				cv::Mat OO2HMD = cv::Mat(m_latestOcTranform);
				cv::Mat IR2HMD = IR2OO * OO2HMD;
				cv::Mat Kin2HMD = Kin2IR * IR2OO * OO2HMD;
				temp = Kin2HMD.t();
				glMultMatrixf((GLfloat*)temp.data);
				drawCoordinateAxis(40);

				glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);
				/*
				glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
				//cv::Mat HMD2Calib = FlipY * m_modelAligner->m_HMD2CalibPattern;
				cv::Mat HMD2Calib = RotateBy180 * m_modelAligner->m_HMD2CalibPattern;
				cv::Mat Kin2Calib = Kin2IR * IR2OO * OO2HMD * HMD2Calib;
				temp = Kin2Calib.t();
				glMultMatrixf((GLfloat*)temp.data);
				//drawCoordinateAxis(40);
				glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);

				glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
				cv::Mat Calib2HMDmodel = m_modelAligner->m_CalibPattern2HMDmodel* TranslateByKeyValues;
				cv::Mat Kin2HMDmodel = Kin2IR * IR2OO * OO2HMD * HMD2Calib * Calib2HMDmodel;
				temp = Kin2HMDmodel.t();
				glMultMatrixf((GLfloat*)temp.data);
				//drawCoordinateAxis(40);
				glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);

				glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
				//temp = Kin2HMDmodel.t();
				temp = Kin2HMD.t();
				glMultMatrixf((GLfloat*)temp.data);
				RenderObject* currModel = m_ModelInfoList[1].mesh;
				glBindVertexArray(currModel->GetVAO());
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, currModel->GetElemBuff());
				glDrawElements(
					GL_TRIANGLES,      // mode
					currModel->GetIndCount(),   // count
					GL_UNSIGNED_INT,   // type
					(void*)0           // element array buffer offset
					);
				glBindVertexArray(0);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
				glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);

				glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
				cv::Mat HMDmodel2FaceModel = m_modelAligner->m_HMDmodel2FaceModel;
				cv::Mat Kin2FaceModel = Kin2IR * IR2OO * OO2HMD * HMD2Calib * Calib2HMDmodel * HMDmodel2FaceModel;
				temp = Kin2FaceModel.t();
				glMultMatrixf((GLfloat*)temp.data);
				//drawCoordinateAxis(20);
				glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);

				glMatrixMode(GL_MODELVIEW); glPushMatrix(); assert(glGetError() == GL_NO_ERROR);
				temp = Kin2FaceModel.t();
				glMultMatrixf((GLfloat*)temp.data);
				RenderObject* currModel2 = m_ModelInfoList[0].mesh;
				glBindVertexArray(currModel2->GetVAO());
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, currModel2->GetElemBuff());
				glDrawElements(
					GL_TRIANGLES,      // mode
					currModel2->GetIndCount(),   // count
					GL_UNSIGNED_INT,   // type
					(void*)0           // element array buffer offset
					);
				glBindVertexArray(0);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
				glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);*/
		}

		glMatrixMode(GL_MODELVIEW); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);
		glMatrixMode(GL_PROJECTION); glPopMatrix(); assert(glGetError() == GL_NO_ERROR);
	}
	//Normal Display mode that renders models and the pointcloud
	else
	{

		if (m_latestOcTrackingState)
		{
			Matx44f Kinect2HMDmodel;
			Matx44f Kinect2FaceModel;
			m_modelAligner->GetTransforms(m_latestOcTranform, m_transxobj, m_transyobj, m_transzobj, m_transx, m_transy, m_transz, Kinect2HMDmodel, Kinect2FaceModel);
			//m_modelAligner->GetTransforms(m_latestOcTranform,0,0,0,0,0,0, Kinect2HMDmodel, Kinect2FaceModel);
			m_ModelInfoList[1].modelTranform = mat4FromFloatArray((float *)Mat(Kinect2HMDmodel).data);
			m_ModelInfoList[0].modelTranform = mat4FromFloatArray((float *)Mat(Kinect2FaceModel).data);
			for (int i = 0; i < 1; i++)
			{
				RenderObject* currModel = m_ModelInfoList[i].mesh;
				if (currModel->GetHasFaces())
				{
					glUseProgram(m_programPointCloud);
					mat4 model_view = m_view * m_ModelInfoList[i].modelTranform;
					setUniformMat4(m_programPointCloud, "modelview", model_view);
					glBindVertexArray(currModel->GetVAO());

					glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, currModel->GetElemBuff());

					////// // Draw the triangles !
					glDrawElements(
						GL_TRIANGLES,      // mode
						currModel->GetIndCount(),   // count
						GL_UNSIGNED_INT,   // type
						(void*)0           // element array buffer offset
						);
					//
					//glUseProgram(0);
					glBindVertexArray(0);
					glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
					glUseProgram(0);
				}
				else
				{

				}

			}
		}
			if (!m_special)
			{
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glFrustum(-0.1, 0.1, -0.1, 0.1, 0.1, 1000);
				glViewport(0, 0, m_WindowWidth, m_WindowHeight);
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				glMultMatrixd(view);

				float temp0 = 0;
				float temp1 = 100000000;
				float temp2 = 0;
				glPointSize(3);

				for (int i = 0; i < m_PointCloudList.size(); i++)
				{
					for (int j = 0; j < m_PointCloudList[i].size(); j++)
					{
						glBegin(GL_POINTS);
						glColor3f(m_PointCloudList[i].points[j].r / 255.0, m_PointCloudList[i].points[j].g / 255.0, m_PointCloudList[i].points[j].b / 255.0);
						glVertex3f(m_PointCloudList[i].points[j].x * 100, m_PointCloudList[i].points[j].y * 100, m_PointCloudList[i].points[j].z * 100);
						glEnd();
					}
				}
				glLoadIdentity();
			}
			else
			{
		/*		double view[] = { m_view[0][0] ,m_view[1][0],m_view[2][0],m_view[3][0],
					m_view[0][1] ,m_view[1][1],m_view[2][1],m_view[3][1],
					m_view[0][2] ,m_view[1][2],m_view[2][2],m_view[3][2],
					m_view[0][3] ,m_view[1][3],m_view[2][3],m_view[3][3] };
				mat4 objTrans = translation3D(vec3(m_transxobj, m_transyobj, m_transzobj)) * rotation3D(vec3(1, 0, 0), m_rotxobj) * rotation3D(vec3(0, 1, 0), m_rotyobj) * rotation3D(vec3(0, 0, 1), m_rotzobj);
				double objTR[] = { objTrans[0][0] ,objTrans[1][0],objTrans[2][0],objTrans[3][0],
					objTrans[0][1] ,objTrans[1][1],objTrans[2][1],objTrans[3][1],
					objTrans[0][2] ,objTrans[1][2],objTrans[2][2],objTrans[3][2],
					objTrans[0][3] ,objTrans[1][3],objTrans[2][3],objTrans[3][3] };
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glFrustum(-0.1, 0.1, -0.1, 0.1, 0.1, 1000);
				glViewport(0, 0, m_WindowWidth, m_WindowHeight);
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				glMultMatrixd(objTR);
				glMultMatrixd(view);
				glColor4f(0.0, 0.0, 0.0, 0.3);
				glScalef(0.05, 0.025, 0.025);
				glBegin(GL_QUADS);
				// front
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(1.0f, 0.0f, 0.0f);
				glVertex3f(1.0f, 1.0f, 0.0f);
				glVertex3f(0.0f, 1.0f, 0.0f);
				// back
				glVertex3f(0.0f, 0.0f, -1.0f);
				glVertex3f(1.0f, 0.0f, -1.0f);
				glVertex3f(1.0f, 1.0f, -1.0f);
				glVertex3f(0.0f, 1.0f, -1.0f);
				// right
				glVertex3f(1.0f, 0.0f, 0.0f);
				glVertex3f(1.0f, 0.0f, -1.0f);
				glVertex3f(1.0f, 1.0f, -1.0f);
				glVertex3f(1.0f, 1.0f, 0.0f);
				// left
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(0.0f, 0.0f, -1.0f);
				glVertex3f(0.0f, 1.0f, -1.0f);
				glVertex3f(0.0f, 1.0f, 0.0f);
				// top
				glVertex3f(0.0f, 1.0f, 0.0f);
				glVertex3f(1.0f, 1.0f, 0.0f);
				glVertex3f(1.0f, 1.0f, -1.0f);
				glVertex3f(0.0f, 1.0f, -1.0f);
				// bottom
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(1.0f, 0.0f, 0.0f);
				glVertex3f(1.0f, 0.0f, -1.0f);
				glVertex3f(0.0f, 0.0f, -1.0f);
				glEnd();
				glLoadIdentity();*/

			}
		}

	
	glutPostRedisplay();
	glutSwapBuffers();
	
}
void ProcessCaptureApp::keyFunc(unsigned char key, int x, int y)
{
	
	switch (key) {
	case '1': m_transx -= 1; printf("m_transx = %f \n", m_transx); ModifyMatrix(-1, 0, 0, m_modelAligner->m_Kinect2IRCamera); break;
	case '2': m_transx += 1; printf("m_transx = %f \n", m_transx); ModifyMatrix(1, 0, 0, m_modelAligner->m_Kinect2IRCamera); break;
	case '3': m_transy -= 1; printf("m_transy = %f \n", m_transy); ModifyMatrix(0, -1, 0, m_modelAligner->m_Kinect2IRCamera); break;
	case '4': m_transy += 1; printf("m_transy = %f \n", m_transy); ModifyMatrix(0, 1, 0, m_modelAligner->m_Kinect2IRCamera); break;
	case '5': m_transz -= 1; printf("m_transz = %f \n", m_transz); ModifyMatrix(0, 0, -1, m_modelAligner->m_Kinect2IRCamera); break;
	case '6': m_transz += 1; printf("m_transz = %f \n", m_transz); ModifyMatrix(0, 0, 1, m_modelAligner->m_Kinect2IRCamera); break;
	case 'q': m_rotx -= 0.1; printf("m_rotx = %f \n", m_rotx); break;
	case 'w': m_rotx += 0.1; printf("m_rotx = %f \n", m_rotx); break;
	case 'e': m_roty -= 1.0f; printf("m_roty = %f \n", m_roty); break;
	case 'r': m_roty += 1.0f; printf("m_roty = %f \n", m_roty); break;
	case 't': m_rotz -= 0.1; printf("m_rotz = %f \n", m_rotz); break;
	case 'y': m_rotz += 0.1; printf("m_rotz = %f \n", m_rotz); break;
	/*case 'z': m_sclx /= 1.01; printf("m_sclx = %f \n", m_sclx); break;
	case 'x': m_sclx *= 1.01; printf("m_sclx = %f \n", m_sclx); break;
	case 'c': m_scly /= 1.01; printf("m_scly = %f \n", m_scly); break;
	case 'v': m_scly *= 1.01; printf("m_scly = %f \n", m_scly); break;
	case 'b': m_sclz /= 1.01; printf("m_sclz = %f \n", m_sclz); break;
	case 'n': m_sclz *= 1.01; printf("m_sclz = %f \n", m_sclz); break;*/
	case 'a': m_transxobj -= 1; printf("m_transxobj = %f \n", m_transxobj);ModifyMatrix(-1, 0, 0, m_modelAligner->m_CalibPattern2HMDmodel); break; 
	case 's': m_transxobj += 1; printf("m_transxobj = %f \n", m_transxobj);ModifyMatrix(1, 0, 0, m_modelAligner->m_CalibPattern2HMDmodel); break; 
	case 'd': m_transyobj -= 1; printf("m_transyobj = %f \n", m_transyobj); ModifyMatrix(0, -1, 0, m_modelAligner->m_CalibPattern2HMDmodel); break;
	case 'f': m_transyobj += 1; printf("m_transyobj = %f \n", m_transyobj);ModifyMatrix(0, 1, 0, m_modelAligner->m_CalibPattern2HMDmodel); break; 
	case 'g': m_transzobj -= 1; printf("m_transzobj = %f \n", m_transzobj); ModifyMatrix(0, 0, -1, m_modelAligner->m_CalibPattern2HMDmodel); break;
	case 'h': m_transzobj += 1; printf("m_transzobj = %f \n", m_transzobj);ModifyMatrix(0, 0, 1, m_modelAligner->m_CalibPattern2HMDmodel); break; 
	//case 'z': m_rotxobj -= 0.1; printf("m_rotx = %f \n", m_rotxobj); break;
	//case 'x': m_rotxobj += 0.1; printf("m_rotx = %f \n", m_rotxobj); break;
	//case 'c': m_rotyobj -= 0.1f; printf("m_roty = %f \n", m_rotyobj); break;
	//case 'v': m_rotyobj += 0.1f; printf("m_roty = %f \n", m_rotyobj); break;
	//case 'b': m_rotzobj -= 0.1; printf("m_rotz = %f \n", m_rotzobj); break;
	//case 'n': m_rotzobj += 0.1; printf("m_rotz = %f \n", m_rotzobj); break;
	case 'z': m_movey = (m_movey > 1)? 0.9f*m_movey:m_movey; printf("m_movey = %f \n", m_movey); break;
	case 'x': m_movey = 1.1*m_movey; printf("m_movey = %f \n", m_movey); break;
	case 'c': m_viewRadius = (m_viewRadius > 1)? 0.9f*m_viewRadius:m_viewRadius; printf("m_viewRadius = %f \n", m_viewRadius); break;
	case 'v': m_viewRadius = 1.1*m_viewRadius; printf("m_viewRadius = %f \n", m_viewRadius); break;
	case 'b': m_viewAngle += 10; printf("m_viewAngle = %f \n", m_viewAngle); break;
	case 'n': m_viewAngle -= 10; printf("m_viewAngle = %f \n", m_viewAngle); break;
	case 'k': m_saveCameraPose = true; break;
	case 'm': break;
	default: printf("Invalid key for static room rendering \n"); break;
	}
}
bool ProcessCaptureApp::createRenderObjects()
{
	if (!m_special)
	{
		vector<float> vertices[3];
		vector<float> colors[3];
		vector<unsigned int> indices[3];
		RenderModelInfo renderInfo[3];
		ReadPLY("./../Models/RafaywithoutRift.ply", vertices[0], colors[0], indices[0]);
		renderInfo[0].mesh = new RenderObject(&vertices[0], &colors[0], &indices[0]);
		renderInfo[0].modelTranform = identity3D();
		//renderInfo.modelTranform = getLookAtCameraMatrix(vec3(0, 0, 0), vec3(0, 0, -1), vec3(0, 1, 0))*translation3D(vec3(0.02, -0.02, -1.5)) *rotation3D(vec3(0, 0, 1), -6.0) * identity3D();
		m_ModelInfoList.push_back(renderInfo[0]);
		ReadPLY("./../Models/chull.ply", vertices[1], colors[1], indices[1]);
		renderInfo[1].mesh = new RenderObject(&vertices[1], &colors[1], &indices[1]);
		renderInfo[1].modelTranform = identity3D();
		m_ModelInfoList.push_back(renderInfo[1]);
		std::cout << "Render Objects Created" << std::endl;
		return true;
	}
	else
	{
		vector<float> vertices[2];
		vector<float> colors[2];
		vector<unsigned int> indices[2];
		RenderModelInfo renderInfo[2];
		ReadPLY("./../Models/FULLbody0WithRiftcleanRemovedTXT.ply", vertices[0], colors[0], indices[0]);
		std::cout << vertices[0].size() << "  " << colors[0].size() << "  " << indices[0].size() << std::endl;
		renderInfo[0].mesh = new RenderObject(&vertices[0], &colors[0], &indices[0]);
		renderInfo[0].modelTranform = identity3D();
		m_ModelInfoList.push_back(renderInfo[0]);
		ReadPLY("./../Models/HeadAttempt1clean.ply", vertices[1], colors[1], indices[1]);
		renderInfo[1].mesh = new RenderObject(&vertices[1], &colors[1], &indices[1]);
		renderInfo[1].modelTranform = mat4(vec4(0.717972, 0.045130 ,- 0.694607, - 0.963954),
			vec4(-0.185171 ,0.974322 ,- 0.128096 ,- 0.286495),
			vec4(0.670990, 0.220590, 0.707893 ,0.010771),
			vec4(0.000000, 0.000000, 0.000000, 1.000000));
		//renderInfo[1].modelTranform = translation3D(vec3(0.25, 0.25, 0.25));//identity3D();
		m_ModelInfoList.push_back(renderInfo[1]);
		return true;
	}
}
bool ProcessCaptureApp::Run(int argc, char* argv[])
{
	init(argc,argv);
	bool liveMode = false;
	if (argc < 2)
	{
		std::cout << "Usage: live" << std::endl;
	}
	else
	{
		if (strcmp(argv[1], "live") == 0)
		{
			liveMode = true;
		}
		else
		{
			liveMode = false;
		}
	}

	if (liveMode)
	{
		createRenderObjects();
		
		if (!initKinect())
		{
			std::cout << "Couldn't Initailize Kinect" << std::endl;
		}
		if (!initOculus())
		{
			std::cout << "Couldn't initialize Oculus" << std::endl;
		}
		if (!initFaceAlignment(false))
		{
			std::cout << "Couldn't initialize Model Aligner" << std::endl;
		}
		glutMainLoop();
	}
	return true;
}

bool ProcessCaptureApp::init(int argc, char ** argv)
{
	if (!this->initGLUT(argc,argv))
	{
		std::cout << "Couldn't Initialize GLUT" << std::endl;
		return false;
	}
	if (!this->initGL())
	{
		std::cout << "Couldn't Initialize GL" << std::endl;
		return false;
	}
	return true;
}

bool ProcessCaptureApp::initGL()
{

	int err = glewInit();
	if ((err != 0))
	{
		cout << "GLEW could not be initialized!" << endl;
		cout << "Error code is: " << err << std::endl;
		return false;
	}
	m_programPointCloud = createShaderProgram(
		"./../Shaders/point_cloud.vert",
		"./../Shaders/point_cloud.frag");
	
	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_MULTISAMPLE_ARB);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glClearColor(0, 0, 0, 1); // Black*/
	m_projection = getPerspectiveMatrix(-0.1, 0.1, -0.1, 0.1, 0.1, 1000);
	m_modelview = identity3D();
	m_view = identity3D();
	//m_modelview = rotation3D(vec3(0, 0, 1), -6.0) * m_modelview;
	//m_modelview = translation3D(vec3(0.02, -0.02, -1.5)) * m_modelview;
	//m_modelview = getLookAtCameraMatrix(vec3(0, 0, 0), vec3(0, 0, -1), vec3(0, 1, 0)) * m_modelview;
	
	setUniformMat4(m_programPointCloud, "projection", m_projection);
	setUniformMat4(m_programPointCloud, "modelview", m_modelview);
	glUseProgram(0);
	return true;
}

ProcessCaptureApp::ProcessCaptureApp(): m_WindowWidth(1024),m_WindowHeight(1024), m_special(false), m_count(0), m_VisualizeTransformations(false)
{
	m_transx = 0;
	m_transy = 0;
	m_transz = 0;
	m_rotx = 0;
	m_roty = 180;
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
	m_saveCameraPose = false;
	m_viewRadius = 13.0;
	m_viewAngle = 160;
	m_movey = 8;
}
void ProcessCaptureApp::ModifyMatrix(float a_x, float a_y, float a_z, cv::Mat &InOut)
{
	InOut.at<float>(0, 3) = InOut.at<float>(0, 3) + a_x;
	InOut.at<float>(1, 3) = InOut.at<float>(1, 3) + a_y;
	InOut.at<float>(2, 3) = InOut.at<float>(2, 3) + a_z;
}
ProcessCaptureApp::~ProcessCaptureApp()
{

}

ProcessCaptureApp* mainApp;
void displayGL()
{
	mainApp->display();
}
void keyGL(unsigned char key, int x, int y)
{
	mainApp->keyFunc(key, x,y);
}
int main(int argc, char** argv)
{

	mainApp = new ProcessCaptureApp;
	mainApp->Run(argc, argv);
	delete mainApp;
	return 0;
}