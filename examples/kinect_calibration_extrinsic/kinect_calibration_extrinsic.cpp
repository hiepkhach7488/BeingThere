#include <stdio.h>
#include <string.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "ImageSequence.h"
#include "opencv2/gpu/gpu.hpp"
#include <fstream>

#define REFINE_MAX_ITER 30
#define REFINE_EPSILON 0.01
#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 960
#define SQUARE_SIZE 4.0

using namespace std;
using namespace cv;

void printMat(CvMat *A)
{	
	int i, j;
	for (i = 0; i < A->rows; i++)
	{
		printf("\n");
		switch (CV_MAT_DEPTH(A->type))
		{
		case CV_32F:
		case CV_64F:
			for (j = 0; j < A->cols; j++)
				printf ("\t\t%8.3f ", (float)cvGetReal2D(A, i, j));
			break;
		case CV_8U:
		case CV_16U:
			for(j = 0; j < A->cols; j++)
				printf ("\t\t%6d",(int)cvGetReal2D(A, i, j));
			break;
		default:
			break;
		}
	}
	printf("\n");
}

void printMat(cv::Mat* A)
{	
	int i, j;
	for (i = 0; i < A->rows; i++)
	{
		printf("\n");
		switch (A->type())
		{
		case CV_32F:
		case CV_64F:
			for (j = 0; j < A->cols; j++)
				printf ("\t\t%8.3f ", (*A).at<float>(i,j));
			break;
		case CV_8U:
		case CV_16U:
			for(j = 0; j < A->cols; j++)
				printf ("\t\t%6d",(*A).at<float>(i,j));
			break;
		default:
			break;
		}
	}
	printf("\n");
}

std::vector<cv::Mat> rVecs;
std::vector<cv::Mat> tVecs;

void writeToFile(std::string file_name_){
	ofstream FILE_OUT;
	FILE_OUT.open(file_name_);

	int size = rVecs.size();

	if(FILE_OUT.is_open()){
		FILE_OUT << size << endl;

		for(int idx=0; idx<size; ++idx){

			for(int i=0; i<3; ++i){
				for(int j=0; j<3; ++j)
					FILE_OUT << rVecs[idx].at<float>(i,j) << " ";
					FILE_OUT << endl;
					//cout << rVecs[idx].at<float>(i,j) << " ";
					//cout << endl;
				}

				for(int i=0; i<3; ++i)
					FILE_OUT << tVecs[idx].at<float>(i) << endl;
					//cout << tVecs[idx].at<float>(i) << endl;

		}
	}

	FILE_OUT.close();
}

void calibrateImages(){
	ImageSequence* imgCapture = new ImageSequence();
	imgCapture->init("C:/Logs/PCL/data/color", ".png", "img_rgb_");
	imgCapture->open();
	imgCapture->captureStart();	

	IplImage* colorImg = imgCapture->image();
	IplImage* grayImg = cvCreateImage(cvSize(colorImg->width, colorImg->height), 8, 1);

	bool found;
	double square_size = 4.0;

	int corner_rows = 6;
	int corner_cols = 7;

	CvPoint2D32f* corners = (CvPoint2D32f*)malloc(sizeof(CvPoint2D32f)*corner_rows*corner_cols);
	CvMat* cam_mat1 = cvCreateMat(3, 3, CV_32FC1 );
	CvMat* cam_mat2 = cvCreateMat(3, 3, CV_32FC1 );
	CvMat* dist_coeff1 = cvCreateMat(5, 1, CV_32FC1 );
	CvMat* dist_coeff2 = cvCreateMat(5, 1, CV_32FC1 );

	CvMat* object_points = cvCreateMat(corner_rows*corner_cols, 3, CV_32FC1 );
	CvMat* point_counts = cvCreateMat(1, 1, CV_32SC1 );

	CvMat* image_points[2];
	image_points[0] = cvCreateMat(corner_rows * corner_cols, 2, CV_32FC1 );
	image_points[1] = cvCreateMat(corner_rows * corner_cols, 2, CV_32FC1 );

	CvMat* rRot = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* rTrans = cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* E = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* F = cvCreateMat( 3, 3, CV_32FC1 );

	cam_mat1 = (CvMat*)cvLoad("C:/Logs/PCL/beingthere/resources/cam_rgb_0.xml", NULL, NULL, NULL);
	dist_coeff1 = (CvMat*)cvLoad("C:/Logs/PCL/beingthere/resources/distort_rgb_0.xml", NULL, NULL, NULL);
	cam_mat2 = cam_mat1;
	dist_coeff2 = dist_coeff1;

	cv::Mat initTrans(1,3, CV_32F);
	initTrans.at<float>(0,0) = 1.5;
	initTrans.at<float>(0,1) = 1.5;
	initTrans.at<float>(0,2) = -0.3;

	int idx = 0; int frame_count_ = 0;
	while(true){
		cvCvtColor(colorImg, grayImg, CV_BGR2GRAY);

		if(grayImg != NULL) {
			int corner_count = 0;
			if(found = cvFindChessboardCorners(grayImg, cvSize(corner_cols, corner_rows), corners, &corner_count, 
				CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_NORMALIZE_IMAGE)) {
					//Find Corner Subpix Accuracy
					cvFindCornerSubPix(grayImg, corners, corner_count, cvSize(11, 11), cvSize(
						-1, -1), cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,
						REFINE_MAX_ITER, REFINE_EPSILON));

					//Draw First ChessBoard
					cvDrawChessboardCorners(grayImg, cvSize(corner_cols, corner_rows), corners, corner_count, found);

					//Good Image To Put Corner
					int offset = corner_rows * corner_cols * 0;
					for (int k = 0; k < corner_rows * corner_cols; k++) {
						//Image Point - Chessboard Corners
						CV_MAT_ELEM(*image_points[idx], float, offset+k, 0 ) = corners[k].x;
						CV_MAT_ELEM(*image_points[idx], float, offset+k, 1 ) = corners[k].y;

						//Object Point - 3D Corners
						CV_MAT_ELEM(*object_points, float, offset+k, 0 ) = square_size * (k % corner_cols);
						CV_MAT_ELEM(*object_points, float, offset+k, 1 ) = square_size * (k / corner_cols);
						CV_MAT_ELEM(*object_points, float, offset+k, 2 ) = 0.0f;
					}

					CV_MAT_ELEM(*point_counts, int, 0, 0 ) = corner_rows * corner_cols;


			} 
			//cvReleaseImage(&grayImg);
		} else {
			printf("Invalid Image \n");
			return;
		}

		if(idx == 1){
			double error = cvStereoCalibrate(object_points, image_points[0], image_points[1], point_counts, cam_mat1, 
					dist_coeff1, cam_mat2, dist_coeff2, cvSize(640, 480), 
                    rRot, rTrans, E, F, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 50,1e-6), CV_CALIB_FIX_INTRINSIC);

			//Rotation & Transposed One
			cv::Mat rotMat(rRot, true);
			cv::Mat tranposeRotMat(rRot, true);

			//Translation
			cv::Mat transMat(rTrans, true);

			//Convert Translation
			for(int i=0; i<3; ++i){
				transMat.at<float>(i) = transMat.at<float>(i)/100;
				transMat.at<float>(i) = initTrans.at<float>(0,i) - transMat.at<float>(i);
			}

			//Transpose Rotatation Mat
			cv::transpose(rotMat, tranposeRotMat);

			rVecs.push_back(tranposeRotMat);
			tVecs.push_back(transMat);

			//printf("\n\n\nRotation:\n");
			//printMat(&tranposeRotMat);
			//printf("\nTranslation:\n");
			//printMat(&transMat);
			//printf("\nEssential:\n");
			////printMat(E);
			//printf("\nRMS Error (px): %f\n", error);
		}

		if(idx == 0) idx = 1;

		cvShowImage("Image", grayImg);
		imgCapture->captureNext();
		colorImg = imgCapture->image();

		frame_count_ ++;
		if(frame_count_ == 200) break;
		cvWaitKey(2);
	}

	writeToFile("ExtrinsicPose.txt");
}

int main (int argc, char **argv)
{
	calibrateImages();
	return 0;

	if(argc != 12) {
		printf("usage: ComputeStereo <skip> <img_dir> <corner_rows> <corner_cols> <num_img> <cam-1-mat> <cam-1-distort> <cam-2-mat> <cam-2-distort> <outDir> <ir-rgb-mode>");
		return 1;
	}

	int skip = atoi(argv[1]);
	char * extDir = argv[2];
	int corner_rows = atoi(argv[3]);
	int corner_cols = atoi(argv[4]);
	int numImages = atoi(argv[5]);
	char * cam1MatFilename = argv[6];
	char * cam1DistortFilename = argv[7];
	char * cam2MatFilename = argv[8];
	char * cam2DistortFilename = argv[9];
	char * outDir = argv[10];
	int irRgbMode = atoi(argv[11]);//0 = rgb, 1 = ir/rgb

	char temp[1024];
	int i, j, k;

	CvPoint2D32f* corners = (CvPoint2D32f*)malloc(sizeof(CvPoint2D32f)*corner_rows*corner_cols);


	int good_images = 0;
	int *good_index = (int*)malloc(sizeof(int)*numImages);

	//scan images to find which are good for both cameras 
	printf("scanning images...\n");
	for(j = 0; j < numImages; j++) {
		int camGood[2] = {0, 0};
		for(k = 0; k < 2; k++) {
			//load image
			sprintf(temp, "%s/img_%s_cam%d_%d.png", extDir, irRgbMode&&i==1?"ir":"rgb",irRgbMode?0:k, j*skip);
			IplImage *img = cvLoadImage(temp, CV_LOAD_IMAGE_GRAYSCALE);
			if(img != NULL) {
				int corner_count = 0;
				if(cvFindChessboardCorners(img, cvSize(corner_cols, corner_rows), corners, &corner_count, 
					CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_NORMALIZE_IMAGE)) {
						camGood[k] = 1;
				} 
				cvReleaseImage(&img);
			} else {
				printf("Cannot find image %s\n", temp);
				return -1;
			}
		}

		good_index[j] = camGood[0] && camGood[1];
		if(good_index[j]) good_images++;

	}

	printf("total images: %d, good images: %d\n", numImages, good_images);

	CvMat* image_points[2];
	CvMat* object_points = cvCreateMat(good_images*corner_rows*corner_cols, 3, CV_32FC1 );
	CvMat* point_counts = cvCreateMat(good_images, 1, CV_32SC1 );
	CvMat* cam_mat1 = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* cam_mat2 = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* dist_coeff1 = cvCreateMat( 5, 1, CV_32FC1 );
	CvMat* dist_coeff2 = cvCreateMat( 5, 1, CV_32FC1 );
	CvMat* rRot = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* rTrans = cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* E = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* F = cvCreateMat( 3, 3, CV_32FC1 );

	for(i = 0; i < 2; i++) {
		int idx = 0;
		image_points[i] = cvCreateMat(good_images*corner_rows*corner_cols, 2, CV_32FC1 );
		for(j = 0; j < numImages; j++) {
			sprintf(temp, "%s/img_%s_cam%d_%d.png", extDir, irRgbMode&&i==1?"ir":"rgb",irRgbMode?0:i, j*skip);
			if(good_index[j]) {

				//load image
				printf("%s\n",temp);
				IplImage *img = cvLoadImage(temp, CV_LOAD_IMAGE_GRAYSCALE);
				if(img != NULL) {
					int corner_count = 0;
					if(cvFindChessboardCorners(img, cvSize(corner_cols, corner_rows), corners, &corner_count, 
						CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_NORMALIZE_IMAGE)) {

							cvFindCornerSubPix(img, corners, corner_count, cvSize(11, 11), cvSize(-1, -1),
								cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, REFINE_MAX_ITER, REFINE_EPSILON));

							int offset = corner_rows*corner_cols*idx;
							for(k = 0; k < corner_rows*corner_cols; k++) {
								CV_MAT_ELEM(*image_points[i], float, offset+k, 0 ) = corners[k].x;
								CV_MAT_ELEM(*image_points[i], float, offset+k, 1 ) = corners[k].y;
								CV_MAT_ELEM(*object_points, float, offset+k, 0 ) = SQUARE_SIZE*(k%corner_cols);
								CV_MAT_ELEM(*object_points, float, offset+k, 1 ) = SQUARE_SIZE*(k/corner_cols);
								CV_MAT_ELEM(*object_points, float, offset+k, 2 ) = 0.0f;
							}
							CV_MAT_ELEM(*point_counts, int, idx, 0 ) = corner_rows*corner_cols;                 
					} else {
						printf("Cannot find corners in image %s, skipping...\n", temp);
						return -1;
					}
					cvReleaseImage(&img);
				} else {
					printf("Cannot find image %s\n", temp);
					return -1;
				}
				idx++;
			} else {
				printf("skipping image %s\n", temp);
			}
		}
	}

	cam_mat1 = (CvMat*) cvLoad(cam1MatFilename, NULL, NULL, NULL);
	dist_coeff1 = (CvMat*) cvLoad(cam1DistortFilename, NULL, NULL, NULL);
	cam_mat2 = (CvMat*) cvLoad(cam2MatFilename, NULL, NULL, NULL);
	dist_coeff2 = (CvMat*) cvLoad(cam2DistortFilename, NULL, NULL, NULL);

	//scale intrinsics focal center and center of projection
	//(input image is twice resolution of that calculated for imported intrinsics matrix)
	CV_MAT_ELEM(*cam_mat1, float, 0, 0 ) *= 2.0;
	CV_MAT_ELEM(*cam_mat1, float, 0, 2 ) *= 2.0;
	CV_MAT_ELEM(*cam_mat1, float, 1, 1 ) *= 2.0;
	CV_MAT_ELEM(*cam_mat1, float, 1, 2 ) *= 2.0;
	CV_MAT_ELEM(*cam_mat2, float, 0, 0 ) *= 2.0;
	CV_MAT_ELEM(*cam_mat2, float, 0, 2 ) *= 2.0;
	CV_MAT_ELEM(*cam_mat2, float, 1, 1 ) *= 2.0;
	CV_MAT_ELEM(*cam_mat2, float, 1, 2 ) *= 2.0;



	double error = cvStereoCalibrate(object_points, image_points[0], image_points[1], point_counts, cam_mat1, dist_coeff1, cam_mat2, dist_coeff2, cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 
		rRot, rTrans, E, F, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 50,1e-6), CV_CALIB_FIX_INTRINSIC);


	printf("\n\n\nRotation:\n");
	printMat(rRot);
	printf("\nTranslation:\n");
	printMat(rTrans);
	printf("\nEssential:\n");
	printMat(E);
	printf("\nRMS Error (px): %f\n", error);

	sprintf(temp, "%s/rot.xml", outDir);
	cvSave(temp, rRot, NULL, NULL, cvAttrList(NULL, NULL));
	sprintf(temp, "%s/trans.xml", outDir);
	cvSave(temp, rTrans, NULL, NULL, cvAttrList(NULL, NULL));
	sprintf(temp, "%s/E.xml", outDir);
	cvSave(temp, E, NULL, NULL, cvAttrList(NULL, NULL));
	sprintf(temp, "%s/F.xml", outDir);
	cvSave(temp, F, NULL, NULL, cvAttrList(NULL, NULL));

	return 0;
}
