#include "data_fusion.h"
#include "beingthere_config.h"

DataFusion::DataFusion(){
	colorLoader = new ImageSequence();
	depthLoader = new ImageSequence();

	colorLoader->init(BEINGTHERE_ROOT_DIR + "resources/multi-cam/data/" , ".png", "snapshot_rgb_");
	colorLoader->open();
	colorLoader->captureStart();
	colorLoader->captureNext();

	depthLoader->init(BEINGTHERE_ROOT_DIR + "resources/multi-cam/data/", ".png", "snapshot_depth_");
	depthLoader->open();
	depthLoader->captureStart();
	depthLoader->captureNext();

	//Initialize Normal Map
	for(int i=0; i<NUM_CAM; ++i)
		nMapSOA[i].create (480* 3, 640);

	calibLoader = new CalibrationLoader();
	
	for(int i=0; i<NUM_CAM; ++i){
		cloud[i].reset(new PointCloud<PointXYZRGB>);
		cloud[i]->points.clear();
	}

	merged_cloud.reset(new PointCloud<PointXYZRGB>);
	merged_cloud->points.clear();
}

DataFusion::~DataFusion(){

}

bool DataFusion::uploadImageData(){
	int cols = depthMat[0].cols;
	int rows = depthMat[0].rows;
	
	for(int i=0; i<NUM_CAM; ++i)
		device_depth[i].upload(depthMat[i].data, cols * 2, rows, cols);

	return true;
}


void DataFusion::createPointCloud(){
	for(int i=0; i<NUM_CAM; ++i){
		beingthere::gpu::createVMap(intrinsic[i], device_depth[i], vMapSOA[i]);
		beingthere::gpu::tranformMaps(vMapSOA[i], nMapSOA[i], g_extrinsicR[i], g_extrinsicT[i], vMapSOA_dst[i], nMapSOA_dst[i]); 
		beingthere::gpu::convert<float4>(vMapSOA_dst[i], vMapAOS[i]);

		float4 *host_ptr = (float4*) malloc(640*480*sizeof(float4));
		vMapAOS[i].download(host_ptr, 640*sizeof(float4));	

		//Load Data To Point Cloud
		uint8_t r(0), g(255), b(15);
		pcl::PointXYZRGB point;	
		for(int x=0; x<480; ++x)
			for(int y=0; y<640; ++y){
				int idx = x * 640 + y;
				point.x = host_ptr[idx].x;
				point.y = host_ptr[idx].y;
				point.z = host_ptr[idx].z;
				r = colorMat[i].at<cv::Vec3b>(x, y)[0];
				g = colorMat[i].at<cv::Vec3b>(x, y)[1];
				b = colorMat[i].at<cv::Vec3b>(x, y)[2];
				uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
					static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				point.rgb = *reinterpret_cast<float*>(&rgb);
				//cloud[i]->points.push_back (point);
				merged_cloud->points.push_back(point);
			}
	}

	for(int i=0; i<NUM_CAM; ++i){
		cloud[i]->width = 640;
		cloud[i]->height = 480;
		cloud[i]->is_dense = false;
	}

	merged_cloud->width = 640;
	merged_cloud->height = 480;
	merged_cloud->is_dense = false;
}

void DataFusion::fuseAllCloud(){

}

bool DataFusion::loadCalibrationData(){
	calibLoader->loadAllCalibrationFiles();
	calibLoader->calculateGlobalCalib();

	for(int i=0; i<NUM_CAM; ++i){
		intrinsic[i] = calibLoader->intrisic[i];
		g_extrinsicR[i] = calibLoader->g_extrinsicR[i];
		g_extrinsicT[i] = calibLoader->g_extrinsicT[i];
		//calibLoader->printMat(g_extrinsicR[i]);
	}

	return true;
}

//Load All Image Data
bool DataFusion::loadImageData(){
	IplImage* temp = 0; 
	for(int i=0; i<NUM_CAM; ++i){
		temp = colorLoader->image();
		assert(temp != 0);
		colorMat[i] = cv::Mat(temp);

		temp = depthLoader->image();
		assert(temp!=0);
		depthMat[i] = cv::Mat(temp);

		colorLoader->captureNext();
		depthLoader->captureNext();
	}

	return true;
}	
