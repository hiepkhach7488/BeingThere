#include "camera.h"

Camera::Camera():_active(true){	

}

Camera::~Camera(){

}

void Camera::clearFrame(){
	delete[] _frame;
}

void* Camera::getCurrentFrame(){
	return _frame;
}

unsigned short* Camera::getCurrentUShortFrame(){
	return _ushort_frame;
}

unsigned char* Camera::getCurrentUCharFrame(){
	return _uchar_frame;
}

void Camera::loadCalibData(string calibFileName){
//load calibration parameters from .calib file

	FILE* calibFile;
	fopen_s(&calibFile, calibFileName.c_str(), "r");
	if (calibFile == NULL){
		cout << "Could not open " << calibFileName.c_str() << endl;
		return;
	}

	fscanf_s(calibFile, "%d %d\n", &_calibration.w, &_calibration.h);
	fscanf_s(calibFile, "%f %f\n", &_calibration.fx, &_calibration.fy);
	fscanf_s(calibFile, "%f %f\n", &_calibration.cx, &_calibration.cy);
	fscanf_s(calibFile, "%f %f %f %f %f\n", &_calibration.kc[0], &_calibration.kc[1], &_calibration.kc[2], &_calibration.kc[3], &_calibration.kc[4]);
	fscanf_s(calibFile, "%f %f %f %f %f %f %f %f %f\n", &_calibration.R[0][0], &_calibration.R[0][1], &_calibration.R[0][2], 
														&_calibration.R[1][0], &_calibration.R[1][1], &_calibration.R[1][2], 
														&_calibration.R[2][0], &_calibration.R[2][1], &_calibration.R[2][2]);
	fscanf_s(calibFile, "%f %f %f\n", &_calibration.T[0], &_calibration.T[1], &_calibration.T[2]);
	//fscanf_s(calibFile, "%f %f %f %f %f %f %f %f %f %f %f %f",&_calibration.Tcolor[0][0], &_calibration.Tcolor[0][1], &_calibration.Tcolor[0][2], &_calibration.Tcolor[0][3],
	//	&_calibration.Tcolor[1][0], &_calibration.Tcolor[1][1], &_calibration.Tcolor[1][2], &_calibration.Tcolor[1][3],
	//	&_calibration.Tcolor[2][0], &_calibration.Tcolor[2][1], &_calibration.Tcolor[2][2], &_calibration.Tcolor[2][3]);
	
	fclose(calibFile);

	/*cout << "w " << _calibration.w << endl;
	cout << "h " << _calibration.h << endl;
	cout << "fx " << _calibration.fx << endl;
	cout << "fy " << _calibration.fy << endl;
	cout << "cx " << _calibration.cx << endl;
	cout << "cy " << _calibration.cy << endl;
	cout << "kc " << _calibration.kc[0] << " " << _calibration.kc[1] << " " << _calibration.kc[2] << " " << _calibration.kc[3] << " " << _calibration.kc[4] << endl;
	cout << "R " << _calibration.R[0][0] << " " << _calibration.R[0][1] << " " << _calibration.R[0][2] << " " << _calibration.R[1][0] << " " << _calibration.R[1][1] << " " << _calibration.R[1][2] << " " << _calibration.R[2][0] << " " << _calibration.R[2][1] << " " << _calibration.R[2][2] << endl;
	cout << "T " << _calibration.T[0] << " " << _calibration.T[1] << " " << _calibration.T[2] << endl;
	cout << "Tcolor " <<	_calibration.Tcolor[0][0] << " " << _calibration.Tcolor[0][1] << " " << _calibration.Tcolor[0][2] << " " << _calibration.Tcolor[0][3] << " " <<
							_calibration.Tcolor[1][0] << " " << _calibration.Tcolor[1][1] << " " << _calibration.Tcolor[1][2] << " " << _calibration.Tcolor[1][3] << " " <<
							_calibration.Tcolor[2][0] << " " << _calibration.Tcolor[2][1] << " " << _calibration.Tcolor[2][2] << " " << _calibration.Tcolor[2][3] << endl;
	cout << endl;*/
	
	createOpenGLMatrices();
	
}

void Camera::createOpenGLMatrices(){

	CalibData c = _calibration;

	//modelview
	_mv = mat4(	vec4(c.R[0][0], c.R[0][1], c.R[0][2], c.T[0]),
				vec4(c.R[1][0], c.R[1][1], c.R[1][2], c.T[1]),
				vec4(c.R[2][0], c.R[2][1], c.R[2][2], c.T[2]),
				vec4(0.0, 0.0, 0.0, 1.0));
	_mv = _mv.transpose(); //opengl stores matrices in column major order
	
	//projection
	float n = 0.5f;
	float f = 4.0f;

	_proj = mat4(	vec4(2.0f*c.fx/c.w,		0.0f,			(2.0f*(c.cx/c.w)-1.0f),	0.0f),
					vec4(0.0f,				2.0f*c.fy/c.h,	(2.0f*(c.cy/c.h)-1.0f),	0.0f),
					vec4(0.0f,				0.0f,			-(n+f)/(n-f),			(2*n*f)/(n-f)),
					vec4(0.0f,				0.0f,			1.0f,					0.0f));
	_proj = _proj.transpose(); //opengl stores matrices in column major order

	//viewport
	_viewport[0] = 0; 
	_viewport[1] = 0;
	_viewport[2] = c.w; 
	_viewport[3] = c.h;
}