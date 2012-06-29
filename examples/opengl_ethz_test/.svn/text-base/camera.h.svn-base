#ifndef CAMERA_H_
#define CAMERA_H_

//#define ZITNICK

#include <iostream>
#include <vector>
#include "algebra.h"

using namespace alg;
using namespace std;

struct CalibData{

	//width and heigth of input image
	int w;
	int h;

	//intrinsics
	float fx; //focal length
	float fy;
	float cx; //principal point
	float cy;
	float kc[5]; //distortion

	//extrinsics
	mat3 R;
	vec3 T;

	//depth cams only: for converting raw depth to meters
	float b; 
	float d_0;

	//mat4 Tcolor; //3x4 matrix for color transformation

};

//base class for real cameras

class Camera
{

public:

	Camera();
	virtual ~Camera();
	virtual void acquire()=0;

	void clearFrame();
	void* getCurrentFrame();
	unsigned short* getCurrentUShortFrame();
	unsigned char* getCurrentUCharFrame();

	void setActive(bool value) { _active = value;};
	bool isActive() { return _active; };

	int getResX() {return _calibration.w;};
	int getResY() {return _calibration.h;};

	mat4 getMv() {return _mv;};
	mat4 getProj() {return _proj;};

	CalibData* getCalibration() {return &_calibration;};
	string getCalibFilename(){ return _calibFilename;};

	void setMv(mat4 mv){ _mv = mv;};

protected:

	void* _frame;
	unsigned short* _ushort_frame;
	unsigned char* _uchar_frame; //for 8bit depth

	//vector<void*> _frames;
	bool _active; //camera on or off
	CalibData _calibration;

	void loadCalibData(string calibFileName);

	//for opengl
	virtual void createOpenGLMatrices();
	mat4 _proj;
	mat4 _mv;
	int _viewport[4];

	string _calibFilename;


private:



};

#endif /*CAMERA_H_*/
