#include "kinect_depth_camera.h"

KinectDepthCamera::KinectDepthCamera(string calibFileName){	
	_calibFilename = calibFileName;
	_serialNumber = "";
	loadCalibData(calibFileName);
}

KinectDepthCamera::~KinectDepthCamera(){
	clearFrame();
	stop();
}

bool KinectDepthCamera::start(){

	cout << "starting kinect depth cam... ";

	//allocate space for frame
	float* buffer = new float[_calibration.w*_calibration.h];
	memset(buffer, 0, _calibration.w*_calibration.h*sizeof(float));
	_frame = (void*) buffer;
	//_frames.push_back(buffer);

	_ushort_frame = (unsigned short*) malloc(_calibration.w*_calibration.h*2);
	_uchar_frame = (unsigned char*) malloc(_calibration.w * _calibration.h);

	//get first kinect
	_kinect = KinectNui::getInstance();
	if (_kinect == NULL){
		cout << " error getting Kinect " << endl;
		return false;
	}
	/*string sn = _kinect->m_SerialNumber;
	_serialNumber = _kinect->m_SerialNumber;
	cout << "ok " << _serialNumber.c_str() << endl;*/

	return true;
}

bool KinectDepthCamera::stop(){
	if (_kinect == NULL){
		return true;
	}

	//nothing to do
	//_kinect->SetLedMode(Kinect::Led_Off);
	_kinect = 0;
	return true;
}

void KinectDepthCamera::acquire(){
	if (_kinect == NULL){
		return;
	}
	//_kinect->ParseDepthBuffer();
	_kinect->Nui_ProcessDepth();
	float* frame = (float*) _frame;
	/*for(int j=0; j<Kinect::KINECT_DEPTH_WIDTH*Kinect::KINECT_DEPTH_HEIGHT; j++){
		frame[j] = float(_kinect->mDepthBuffer[j]);
	}*/
	_kinect->copyDepth(frame);
	_kinect->copyDepth(_ushort_frame);
	_kinect->copyDepth(_uchar_frame);
}
