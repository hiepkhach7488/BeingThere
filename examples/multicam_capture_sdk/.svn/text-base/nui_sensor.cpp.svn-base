#include "nui_sensor.h"

NuiSensor::NuiSensor(){
	HRESULT hr = NuiCreateSensorByIndex(0,  &m_pNuiSensor);
	if(m_pNuiSensor == NULL) {
		cout << "Cannot Create Nui Sensor" << endl;
		getchar(); 
		exit(0);
	}

	m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_hNextColorFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_hEvNuiProcessStop = CreateEvent( NULL, FALSE, FALSE, NULL );

	DWORD nuiFlags = NUI_INITIALIZE_FLAG_USES_DEPTH |  NUI_INITIALIZE_FLAG_USES_COLOR;
	hr = m_pNuiSensor->NuiInitialize( nuiFlags );
}

NuiSensor::NuiSensor(int camIdx){
	HRESULT hr = NuiCreateSensorByIndex(camIdx,  &m_pNuiSensor);

	if(m_pNuiSensor == NULL) {
		cout << "Cannot Create Nui Sensor" << endl;
		getchar(); 
		exit(0);
	}

	m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_hNextColorFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_hEvNuiProcessStop = CreateEvent( NULL, FALSE, FALSE, NULL );

	DWORD nuiFlags = NUI_INITIALIZE_FLAG_USES_DEPTH |  NUI_INITIALIZE_FLAG_USES_COLOR;
	//DWORD nuiFlags = NUI_INITIALIZE_FLAG_USES_DEPTH;
	hr = m_pNuiSensor->NuiInitialize( nuiFlags );
}

NuiSensor::~NuiSensor(){
	if ( m_pNuiSensor )
	{
		m_pNuiSensor->NuiShutdown( );
	}

	if ( m_hNextDepthFrameEvent && ( m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( m_hNextDepthFrameEvent );
		m_hNextDepthFrameEvent = NULL;
	}

	if ( m_hNextColorFrameEvent && ( m_hNextColorFrameEvent != INVALID_HANDLE_VALUE ) )
	{
		CloseHandle( m_hNextColorFrameEvent );
		m_hNextColorFrameEvent = NULL;
	}

	if ( m_pNuiSensor )
	{
		m_pNuiSensor->Release();
		m_pNuiSensor = NULL;
	}
}

bool NuiSensor::openStreams(){
	HRESULT hr;
	hr = m_pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		m_hNextColorFrameEvent,
		&m_pVideoStreamHandle );

	if ( FAILED( hr ) )
	{
		cout << "Error: Cannot Open Color Stream" << endl;
		return hr;
	}

	hr = m_pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		m_hNextDepthFrameEvent,
		&m_pDepthStreamHandle );

	if ( FAILED( hr ) )
	{
		cout << "Error: Cannot Open Depth Stream" << endl;
		return hr;
	}

	return true;
}

void NuiSensor::Nui_GotColorAlert( )
{
	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame( m_pVideoStreamHandle, 0, &imageFrame ); 

	if ( FAILED( hr ) )
	{
		return;
	}

	INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );

	if ( LockedRect.Pitch != 0 )
	{
		uploadColorToTexture((void*)LockedRect.pBits);
	}
	else
	{
		cout<< "Buffer length of received texture is bogus\r\n" << endl;
	}

	pTexture->UnlockRect( 0 );

	m_pNuiSensor->NuiImageStreamReleaseFrame( m_pVideoStreamHandle, &imageFrame );
}

void NuiSensor::Nui_GotDepthAlert( )
{
	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(
		m_pDepthStreamHandle,
		0,
		&imageFrame );

	if ( FAILED( hr ) )
	{
		return;
	}

	INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );

	if ( 0 != LockedRect.Pitch )
	{
		DWORD frameWidth, frameHeight;

		NuiImageResolutionToSize( imageFrame.eResolution, frameWidth, frameHeight );

		uploadDepthToTexture((void*)LockedRect.pBits);
	}
	else
	{
		cout<< "Buffer length of received texture is bogus\r\n" << endl;
	}

	pTexture->UnlockRect( 0 );

	m_pNuiSensor->NuiImageStreamReleaseFrame( m_pDepthStreamHandle, &imageFrame );
}

void NuiSensor::uploadDepthToTexture(void* depthData){
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, depthTex);

	if(depthData != 0) {
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depthTemp);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, depthPBO);
		void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
		if(ptr != NULL) {
			memcpy(ptr, depthData, 640*480*2);
			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_GREEN, GL_UNSIGNED_SHORT, 0);	
		}
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}

void NuiSensor::uploadColorToTexture(void* colorData){
	//Draw To Texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, colorTex);

	//If Having New Data -> Upload Data
	if(colorData != 0) {
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, pImage);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, colorPBO);
		void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
		if(ptr != NULL) {
			memcpy(ptr, colorData, 640*480*4);
			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGBA, GL_UNSIGNED_BYTE, 0);

		}
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}

void NuiSensor::initTexturePBO(){
	//create textures
	unsigned char * dummyTex = (unsigned char *)malloc(640*480*4);
	memset(dummyTex, 0, 640*480*4);
	unsigned short* dummyDepth = (unsigned short*) malloc(640*480*2);
	memset(dummyDepth, 0, 640*480*2);

	//create textures
	glGenTextures(1, &colorTex);
	glGenTextures(1, &depthTex);

	glBindTexture(GL_TEXTURE_2D, colorTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 640, 480, 0, GL_RGBA, GL_UNSIGNED_BYTE, dummyTex);

	glBindTexture(GL_TEXTURE_2D, depthTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, 640, 480, 0, GL_GREEN, GL_UNSIGNED_SHORT, dummyDepth);

	glBindTexture(GL_TEXTURE_2D, 0);

	//create PBOs
	glGenBuffers(1, &colorPBO);
	glGenBuffers(1, &depthPBO);

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, colorPBO);
	glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*4, dummyTex, GL_DYNAMIC_DRAW);

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, depthPBO);
	glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*2, dummyDepth, GL_DYNAMIC_DRAW);

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
}

bool NuiSensor::waitNextFrame(){
	const int numEvents = 2;
	HANDLE hEvents[numEvents] = {m_hNextDepthFrameEvent, m_hNextColorFrameEvent};
	//const int numEvents = 1;
	//HANDLE hEvents[numEvents] = {m_hNextDepthFrameEvent};

	int    nEventIdx;

	bool depthReady = false, colorReady = false;

	//TODO: PUT TIMEOUT CLOCKS
	while (!depthReady && !colorReady)
	{
		// Wait for any of the events to be signalled
		nEventIdx = WaitForMultipleObjects( numEvents, hEvents, FALSE, 100 );

		// Process signal events
		switch ( nEventIdx )
		{
		case WAIT_TIMEOUT:
			break;

		case WAIT_OBJECT_0:
			if(!depthReady){
				Nui_GotDepthAlert();
				depthReady = true;
			}
			cout << "Depth Ready" << endl;
			break;

		case WAIT_OBJECT_0 + 1:
			if(!colorReady){
				Nui_GotColorAlert();
				colorReady = true; cout << "Color Ready" << endl;
			}
			
			break;
		}
	}

	return true;
}