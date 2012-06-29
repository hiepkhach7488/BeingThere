#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <pcl/visualization/image_viewer.h>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include <cv.h>
#include <highgui.h>

#include "multicam_fusion.h"

int main(int argc, char* argv[]){
	IplImage *depthImg0 = cvLoadImage("D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_depth_3.png", CV_LOAD_IMAGE_UNCHANGED);
	DeviceArray2D<unsigned short> depth_raw_;	

	MulticamFusion* multiFusion = new MulticamFusion();
	while(true){
		depth_raw_.upload(depthImg0->imageData, 640*2, 480, 640);
		multiFusion->integrateTSDF(depth_raw_);
		//multiFusion->show(&multiFusion->getViewerPose());
		multiFusion->show();
	}
	return 0;
}