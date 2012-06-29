#include "scene.h"

Scene::Scene(): 
_currentFrame(0),
_depthCam(NULL),
_colorCam(NULL){


}

Scene::~Scene(){
	clear();

}

void Scene::clear() {
	delete _depthCam;
	_depthCam = NULL;
	delete _colorCam;
	_colorCam = NULL;

}

//void Scene::setPgColorCam(string calibFileName){
//	PGColorCamera* colorCam = new PGColorCamera(calibFileName);
//	colorCam->start();
//	_colorCam = colorCam;
//}

void Scene::setKinectColorCam(string calibFileName){
	KinectColorCamera* colorCam = new KinectColorCamera(calibFileName);
	colorCam->start();
	_colorCam = colorCam;
}

void Scene::setKinectDepthCam(string calibFileName){
	KinectDepthCamera* depthCam = new KinectDepthCamera(calibFileName);
	depthCam->start();
	_depthCam = depthCam;

}

Resolution Scene::getDepthResolution() {

	Resolution r;
	if (_depthCam != NULL) {
		r.x = _depthCam->getResX();
		r.y = _depthCam->getResY();
	} else {
		r.x = 0;
		r.y = 0;
	}
	return r;
}

Resolution Scene::getColorResolution() {

	Resolution r;
	if (_colorCam != NULL){
		r.x = _colorCam->getResX();
		r.y = _colorCam->getResY();
	} else {
		r.x = 0;
		r.y = 0;
	}
	return r;
}
