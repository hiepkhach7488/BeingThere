#include <stdio.h>
#include <string.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#define REFINE_MAX_ITER 30
#define REFINE_EPSILON .01
#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 960
#define SQUARE_SIZE 4.0

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
				printf ("\t\t%8.8f ", (float)cvGetReal2D(A, i, j));
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


int main (int argc, char **argv)
{
	if(argc < 8) {
		printf("usage: Compute <skip> <out_dir> <num_cameras> <corner_rows> <corner_cols> <int_dir1> <rgb_images1> <ir_images1> <int_dir2> ...\n");
		return 1;
	}

	int skip = atoi(argv[1]);
	char * outDir = argv[2];
	int numCameras = atoi(argv[3]);
	int corner_rows = atoi(argv[4]);
	int corner_cols = atoi(argv[5]);
	int argBase = 6;
	if(argc != argBase+3*numCameras) {
		printf("Expected %d intrinsics entries for %d cameras.\n", numCameras, numCameras);
		return -1;
	}

	char temp[1024];
	int i, j, k, type;  

	CvMat* cam_mat = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* dist_coeff = cvCreateMat( 5, 1, CV_32FC1 );
	CvPoint2D32f* corners = (CvPoint2D32f*) malloc(sizeof(CvPoint2D32f)*corner_rows*corner_cols);
	for(type = 0; type <= 1; type++) {
		for(i = 0; i < numCameras; i++) {
			int numSkipped = 0;
			int numImages = atoi(argv[argBase+i*3+(type==0?1:2)]);
			char * intDir = argv[argBase+i*3];
			CvMat* image_points = cvCreateMat(numImages*corner_rows*corner_cols, 2, CV_32FC1 );
			CvMat* object_points = cvCreateMat(numImages*corner_rows*corner_cols, 3, CV_32FC1 );
			CvMat* point_counts = cvCreateMat(numImages, 1, CV_32SC1 );
			for(j = 0; j < numImages; j++) {
				//load image
				sprintf(temp, "%s/img_%s_%d.png", intDir, type==0?"rgb":"ir", j*skip);
				printf("loading %s...\n",temp);

				IplImage *img = cvLoadImage(temp, CV_LOAD_IMAGE_GRAYSCALE);
				if(img != NULL) {
					int corner_count = 0;
					if(cvFindChessboardCorners(img, cvSize(corner_cols, corner_rows), corners, &corner_count, 
						CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_NORMALIZE_IMAGE)) {

							cvFindCornerSubPix(img, corners, corner_count, cvSize(11, 11), cvSize(-1, -1),
								cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, REFINE_MAX_ITER, REFINE_EPSILON));

							int offset = (j-numSkipped)*corner_rows*corner_cols;
							for(k = 0; k < corner_rows*corner_cols; k++) {
								CV_MAT_ELEM(*image_points, float, offset+k, 0 ) = corners[k].x;
								CV_MAT_ELEM(*image_points, float, offset+k, 1 ) = corners[k].y;
								CV_MAT_ELEM(*object_points, float, offset+k, 0 ) = SQUARE_SIZE*(k%corner_cols);
								CV_MAT_ELEM(*object_points, float, offset+k, 1 ) = SQUARE_SIZE*(k/corner_cols);
								CV_MAT_ELEM(*object_points, float, offset+k, 2 ) = 0.0f;
							}
							CV_MAT_ELEM(*point_counts, int, j-numSkipped, 0 ) = corner_rows*corner_cols;

					} else {
						printf("Cannot find corners in image %s, skipping\n", temp);
						numSkipped++;
						//return -1;
					}


					cvReleaseImage(&img);
				} else {
					printf("Cannot find image %s\n", temp);
					return -1;
				}
			}

			//resize data for skipped images
			if(numSkipped > 0) {
				numImages -= numSkipped;
				CvMat* image_points2 = cvCreateMat(numImages*corner_rows*corner_cols, 2, CV_32FC1 );
				CvMat* object_points2 = cvCreateMat(numImages*corner_rows*corner_cols, 3, CV_32FC1 );
				CvMat* point_counts2 = cvCreateMat(numImages, 1, CV_32SC1 );

				for(j = 0; j < numImages; j++) {
					int offset = j*corner_rows*corner_cols;
					for(k = 0; k < corner_rows*corner_cols; k++) {
						CV_MAT_ELEM(*image_points2, float, offset+k, 0 ) = CV_MAT_ELEM(*image_points, float, offset+k, 0 );
						CV_MAT_ELEM(*image_points2, float, offset+k, 1 ) = CV_MAT_ELEM(*image_points, float, offset+k, 1 );
						CV_MAT_ELEM(*object_points2, float, offset+k, 0 ) = CV_MAT_ELEM(*object_points, float, offset+k, 0 );
						CV_MAT_ELEM(*object_points2, float, offset+k, 1 ) = CV_MAT_ELEM(*object_points, float, offset+k, 1 );
						CV_MAT_ELEM(*object_points2, float, offset+k, 2 ) = CV_MAT_ELEM(*object_points, float, offset+k, 2 );
					}
					CV_MAT_ELEM(*point_counts2, int, j, 0 ) = CV_MAT_ELEM(*point_counts, int, j, 0 );
				}
				image_points = image_points2;
				object_points = object_points2;
				point_counts = point_counts2;
			}

			printf("Calculating %s calibration for camera %d\n",type==0?"RGB":"IR",i);

			//compute intrinsics
			double error = cvCalibrateCamera2(object_points, image_points, point_counts, cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), cam_mat, dist_coeff, 
				NULL, NULL, CV_CALIB_FIX_K3);

			//scale matrices by 0.5 (convert from 1280x960->VGA)
			CV_MAT_ELEM(*cam_mat, float, 0, 0 ) *= 0.5;
			CV_MAT_ELEM(*cam_mat, float, 0, 2 ) *= 0.5;
			CV_MAT_ELEM(*cam_mat, float, 1, 1 ) *= 0.5;
			CV_MAT_ELEM(*cam_mat, float, 1, 2 ) *= 0.5;

			printf("%s Camera %d:\n", type==0?"RGB":"IR",i);
			printf("\tCamera Matrix:\n");
			printMat(cam_mat);
			printf("\tDistort Coeff:\n");
			printMat(dist_coeff);
			printf("\tRMS Error (px): %f\n\n", error);



			//save matrices
			sprintf(temp, "%s/cam_%s_%d.xml", outDir, type==0?"rgb":"ir", i);
			cvSave(temp, cam_mat, NULL, NULL, cvAttrList(NULL, NULL));
			sprintf(temp, "%s/distort_%s_%d.xml", outDir, type==0?"rgb":"ir", i);
			cvSave(temp, dist_coeff, NULL, NULL, cvAttrList(NULL, NULL));

			cvReleaseMat(&image_points);
			cvReleaseMat(&object_points);
			cvReleaseMat(&point_counts);

		}
	}
	free(corners);

	cvReleaseMat(&cam_mat);
	cvReleaseMat(&dist_coeff);


	return 0;
}
