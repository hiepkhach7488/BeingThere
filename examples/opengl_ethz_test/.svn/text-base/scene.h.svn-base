#ifndef SCENE_H_
#define SCENE_H_

//scene has one depth cam and one color cam

#include "kinect_color_camera.h"
#include "kinect_depth_camera.h"
//#include "pg_color_camera.h"

struct Resolution {
	int x;
	int y;
};

class Scene
{

public:

	Scene();
	virtual ~Scene();

	void setKinectDepthCam(string calibFilename); 
	void setKinectColorCam(string calibFilename);
	//void setPgColorCam(string calibFilename);

	Camera* getColorCam() { return _colorCam; }
	Camera* getDepthCam() { return _depthCam; }

	void clear();
	Resolution getDepthResolution();
	Resolution getColorResolution();

protected:

private:
	int _currentFrame;
	Camera* _colorCam;
	Camera* _depthCam;

};

#endif /*SCENE_H_*/
