#include "kinect_color_camera.h"


KinectColorCamera::KinectColorCamera(string calibFileName){	
	_calibFilename = calibFileName;
	_serialNumber = "";
	loadCalibData(calibFileName);
}

KinectColorCamera::~KinectColorCamera(){
	clearFrame();
	stop();
}

bool KinectColorCamera::start(){

	cout << "starting kinect color cam... ";

	//allocate space for frame
	unsigned char* buffer = new unsigned char[_calibration.w*_calibration.h*3];
	memset(buffer, 0, _calibration.w*_calibration.h*3*sizeof(unsigned char));
	_frame = (void*) buffer;

	//get first kinect
	_kinect = KinectNui::getInstance();
	if (_kinect == NULL){
		cout << " error getting Kinect " << endl;
		return false;
	}
	/*string sn = _kinect->m_SerialNumber;
	_serialNumber = _kinect->m_SerialNumber;

	cout << "ok" << endl;*/
	return true;

}

bool KinectColorCamera::stop(){
	if (_kinect == NULL){
		return true;
	}

	//nothing to do
	//_kinect->SetLedMode(Kinect::Led_Off);
	_kinect = 0;
	return true;
}

void KinectColorCamera::acquire(){
	if (_kinect == NULL){
		return;
	}
	//_kinect->ParseColorBuffer();
	_kinect->Nui_ProcessColor();
	unsigned char* frame = (unsigned char*) _frame;
	//memcpy(frame, _kinect->mColorBuffer, Kinect::KINECT_DEPTH_WIDTH*Kinect::KINECT_DEPTH_HEIGHT*3*sizeof(unsigned char));
	_kinect->copyColor(frame);
}
