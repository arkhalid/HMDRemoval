#define NUM_IMAGES 15
//#define CORNER_COLS 3
//#define CORNER_ROWS 4
#define CORNER_COLS 9
#define CORNER_ROWS 6
#define REFINE_MAX_ITER 100
#define REFINE_EPSILON .001
#define SQUARE_SIZE 9.0 //doesn't matter
#define IMG_ROWS 480
#define IMG_COLS 640

#pragma once
#include <stdlib.h>
#include <GL/glew.h>
#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glut.h>

#include "opencv2/opencv.hpp"
using namespace cv;

// Globals:
CvMat* cam_mat = cvCreateMat(3, 3, CV_32FC1);

CvMat* dist_coeff = cvCreateMat(5, 1, CV_32FC1);
CvPoint2D32f* corners = (CvPoint2D32f*)malloc(sizeof(CvPoint2D32f)*CORNER_ROWS*CORNER_COLS);
CvMat* image_points = cvCreateMat(NUM_IMAGES*CORNER_ROWS*CORNER_COLS, 2, CV_32FC1);
CvMat* object_points = cvCreateMat(NUM_IMAGES*CORNER_ROWS*CORNER_COLS, 3, CV_32FC1);
CvMat* point_counts = cvCreateMat(NUM_IMAGES, 1, CV_32SC1);
Mat hist1, hist2;//Older Images (Temporal Filtering)
int width, height;

bool saveImage = false, useImage = false, once = true;
int counter = 0;
VideoCapture cap; // open the default camera

void printMat(CvMat *A) {
	int i, j;
	for (i = 0; i < A->rows; i++) {
		printf("\n");
		switch (CV_MAT_DEPTH(A->type)) {
		case CV_32F:
		case CV_64F:
			for (j = 0; j < A->cols; j++)
				printf("\t\t%8.8f ", (float)cvGetReal2D(A, i, j));
			break;
		case CV_8U:
		case CV_16U:
			for (j = 0; j < A->cols; j++)
				printf("\t\t%6d", (int)cvGetReal2D(A, i, j));
			break;
		default:
			break;
		}
	}
	printf("\n");
}



void customImwrite(char* filename, Mat img) {
	IplImage *iplimg = new IplImage(img);
	cvSaveImage(filename, iplimg);
}

void initializeglut(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitWindowSize(800, 600);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(120, 100);
	glutCreateWindow("Kinect Rift Calib");

}

void key(unsigned char k, int x, int y) {
	switch (k) {
	case 27: exit(0);
	case 's': saveImage = true; break;
	case 'u': useImage = true; break;
	case 'r': counter--; useImage = true; break;
	default: break;
	}
}

void intrinsicCalibrate(Mat currImg) {
	IplImage *img = new IplImage(currImg);

	int corner_count = 0;
	width = img->width;
	height = img->height;
	if (cvFindChessboardCorners(img, cvSize(CORNER_COLS, CORNER_ROWS), corners, &corner_count,
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE)) {
		cvFindCornerSubPix(img, corners, corner_count, cvSize(11, 11), cvSize(-1, -1),
			cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, REFINE_MAX_ITER, REFINE_EPSILON));

		cvDrawChessboardCorners(img, cvSize(CORNER_COLS, CORNER_ROWS), corners, CORNER_ROWS*CORNER_COLS, 1);
		cvShowImage("Image", img);

		int offset = (counter)*CORNER_ROWS*CORNER_COLS;
		for (int k = 0; k < CORNER_ROWS*CORNER_COLS; k++) {
			CV_MAT_ELEM(*image_points, float, offset + k, 0) = corners[k].x;
			CV_MAT_ELEM(*image_points, float, offset + k, 1) = corners[k].y;
			CV_MAT_ELEM(*object_points, float, offset + k, 0) = SQUARE_SIZE*(k%CORNER_COLS);
			CV_MAT_ELEM(*object_points, float, offset + k, 1) = SQUARE_SIZE*(k / CORNER_COLS);
			CV_MAT_ELEM(*object_points, float, offset + k, 2) = 0.0f;
		}
		CV_MAT_ELEM(*point_counts, int, counter, 0) = CORNER_ROWS*CORNER_COLS;

	}
	else {
		printf("Cannot find corners \n");
		counter--;
	}
}

void calculateIntrinsics() {
	double error = cvCalibrateCamera2(object_points, image_points, point_counts, cvSize(width, height), cam_mat, dist_coeff, NULL, NULL, CV_CALIB_FIX_K3);
	printf("\tCamera Matrix:\n");
	printMat(cam_mat);
	printf("\tDistort Coeff:\n");
	printMat(dist_coeff);
	printf("\tRMS Error (px): %f\n\n", error);

	//save matrices
	char temp[1024];

	sprintf(temp, "./cam_rgb_0.xml");
	cvSave(temp, cam_mat, NULL, NULL, cvAttrList(NULL, NULL));
	sprintf(temp, "./distort_rgb_0.xml");
	cvSave(temp, dist_coeff, NULL, NULL, cvAttrList(NULL, NULL));
	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);
	free(corners);

	cvReleaseMat(&cam_mat);
	cvReleaseMat(&dist_coeff);

}

void display() {
	Mat frame;
	cap >> frame; // get a new frame from camera
	cvtColor(frame, frame, CV_BGR2GRAY);
	//GaussianBlur(frame, frame, Size(3, 3), 1.0, 1.0);
	imshow("frame", frame);
	Mat uImg = frame;
	if (saveImage) {
		customImwrite("uImg.png", uImg);
		customImwrite("frame.png", frame);
		saveImage = false;
	}

	imshow("uImg", uImg);
	if (counter < NUM_IMAGES) {
		if (useImage) {
			intrinsicCalibrate(uImg);
			counter++;
			printf("Counter: %d \n", counter);
			useImage = false;
		}
	}
	else { // When we have got all the images that are good for calibration
		calculateIntrinsics();
		int x;
		std::cin >> x;
		exit(0);
	}

	hist2 = hist1;
	hist1 = uImg;
	if (once) { printf("%d %d \n", frame.rows, frame.cols); once = false; }
	waitKey(1);

}

bool initCamera() {

	bool toRet = true;
	cap = VideoCapture(2);
 if (!cap.isOpened())  // check if camera is available
		toRet = false;
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, IMG_ROWS);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, IMG_COLS);
	int tempc = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int tempr = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);


	hist1 = Mat(tempr, tempc, CV_8UC1, cvScalar(0));
	hist2 = Mat(tempr, tempc, CV_8UC1, cvScalar(0));


//	oimg = Mat(tempr / 2, tempc, CV_8UC1, cvScalar(0));
//	eimg = Mat(tempr / 2, tempc, CV_8UC1, cvScalar(0));
//	int temp = (int)cap.get(CV_CAP_PROP_FOURCC);
	//printf("Format: %d \n", temp);
	return true;
}

int main(int argc, char* argv[]) {

	initializeglut(argc, argv);
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		printf("Failed to init GLEW.\n");
		return 0;
	}
	if (!initCamera()) return 0;
	printf("Working \n");
	//These lines are used to supply a guess to the calibration
	/* CV_MAT_ELEM(*cam_mat, float, 0, 0 ) = 600;
	CV_MAT_ELEM(*cam_mat, float, 1, 1 ) = 600;
	CV_MAT_ELEM(*cam_mat, float, 0, 2 ) = 320;
	CV_MAT_ELEM(*cam_mat, float, 1, 2 ) = 240;*/
	glutDisplayFunc(display);
	glutIdleFunc(display);
	glutKeyboardFunc(key);
	printf("Working \n");
	glutMainLoop();
	cvWaitKey(0);
	return 0;
}
