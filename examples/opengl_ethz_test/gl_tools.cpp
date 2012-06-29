
#include "gl_tools.h"

#include <GL/glut.h>


void GLTools::pushMatrices(){
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
}

void GLTools::popMatrices(){
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void GLTools::drawTriangle() {
	glBegin (GL_TRIANGLES); 
	glVertex3i(-1, 3, 0);
	glVertex3i(-1, -1, 0);
	glVertex3i(3, -1, 0);
	glEnd ();
}

void GLTools::drawOrigin() {

	glBegin(GL_LINES);
	glColor3f(0.0, 1.0, 0.0);
	glVertex3i(0, 0, 0);
	glVertex3i(1, 0, 0);
	
	glColor3f(1.0, 0.0, 0.0);
	glVertex3i(0, 0, 0);
	glVertex3i(0, 1, 0);
	
	glColor3f(0.0, 0.0, 1.0);
	glVertex3i(0, 0, 0);
	glVertex3i(0, 0, 1);
	glEnd();
}

void GLTools::drawMesh(GridMesh* mesh){
	glBindBuffer(GL_ARRAY_BUFFER, mesh->getVbo());
	glVertexPointer(3, GL_FLOAT, 0, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->getIbo());
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawElements(GL_TRIANGLES, mesh->getNrElements(), GL_UNSIGNED_INT, 0);
	glDisableClientState(GL_VERTEX_ARRAY);
	//glFinish();
}

void GLTools::drawText(char* str, int posX, int posY, int windowWidth, int windowHeight){

	pushMatrices();
	glOrtho (0, windowWidth, 0, windowHeight, 0.0, 1.0);
	glRasterPos2i(posX, posY);
	for(int i=0; i<strlen(str); i++){
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str[i]);
	}
	popMatrices();
}

void GLTools::writeDat(GLuint texId, char* name, int w, int h){
	cout << "writing dat...";
	float* buffer = new float[w*h];
	glBindTexture(GL_TEXTURE_2D, texId);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, buffer);
	glBindTexture(GL_TEXTURE_2D, 0);

	float* flipBuffer = new float[w*h];
	memcpy(flipBuffer, buffer, w*h*sizeof(float));
	for(int j=0; j<h; j++){
		memcpy(&buffer[w*(h-1) - j*w], &flipBuffer[j*w], w*sizeof(float));
	}
	delete[] flipBuffer;

	ofstream file;
	file.open(name, ios::out | ios::binary);
	//file.write((const char*) buffer, w*h*sizeof(float));
	file.write((const char*) buffer, w*h*sizeof(float));
	file.close();
	delete[] buffer;

	cout << "done" << endl;
}

void GLTools::writeTxt(float* buffer, char* name, int w, int h){
//write buffer to text file

	cout << "writing dat...";
	ofstream file;
	file.open(name, ios::out | ios::binary);
	for(int i=0; i<h; i++){
		for(int j=0; j<w; j++){
			file << buffer[i*w+j] << " ";
		}
		file << endl;
	}
	file.close();

	cout << "done" << endl;
}

void GLTools::writeTxt(GLuint texId, char* name, int w, int h){
	float* bufferF = new float[w*h];
	glBindTexture(GL_TEXTURE_2D, texId);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, bufferF);
	glBindTexture(GL_TEXTURE_2D, 0);
	writeTxt(bufferF, name, w, h);
	delete[] bufferF;
}

void GLTools::writePPMColor(unsigned char* bufferC, char* name, int w, int h){
	ofstream outfile(name);
	outfile << "P3" << endl;
	outfile << w << endl;
	outfile << h << endl;
	outfile << 255 <<endl;
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w*3; j+=3){
			outfile << int(bufferC[i*w*3+j]) << " " << int(bufferC[i*w*3+j+1]) << " " << int(bufferC[i*w*3+j+2]) << " ";
		}
		outfile << endl;
	}
	outfile.close();
}

void GLTools::writePPMColor(GLuint texId, char* name, int w, int h){
	
	unsigned char* bufferC = new unsigned char[w*h*3];

	glBindTexture(GL_TEXTURE_2D, texId);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, bufferC);
	glBindTexture(GL_TEXTURE_2D, 0);

	writePPMColor(bufferC, name, w, h);

	delete[] bufferC;
}

void GLTools::writePPMColorF(GLuint texId, char* name, int w, int h){

	float* bufferF = new float[w*h*3];

	glBindTexture(GL_TEXTURE_2D, texId);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, bufferF);
	glBindTexture(GL_TEXTURE_2D, 0);

	writePPMColorF(bufferF, name, w, h);

	delete[] bufferF;
}

void GLTools::writePPMColorF(float* bufferF, char* name, int w, int h){

	unsigned char* bufferC = new unsigned char[w*h*3];

	for(int i=0; i<w*h*3; i++){
		float v = bufferF[i]*255.0f;
		bufferC[i] = (unsigned char)std::max(0.0f, std::min(255.0f, v));
	}

	writePPMColor(bufferC, name, w, h);

	delete[] bufferC;
}

void GLTools::writePPM(unsigned char* buffer, char* name, int w, int h){

	ofstream outfile(name);
	outfile << "P3" << endl;
	outfile << w << endl;
	outfile << h << endl;
	outfile << 255 <<endl;	
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++){			
			outfile << int(buffer[i*w+j]) << " " << int(buffer[i*w+j]) << " " << int(buffer[i*w+j]) << " ";
		}
		outfile << endl;
	}
	outfile.close();
}

void GLTools::writePPM(float* bufferF, char* name, int w, int h){
	unsigned char* bufferC = new unsigned char[w*h];
	for(int j=0; j<w*h; j++){
		//float v = bufferF[j]*255.0;
		float v = bufferF[j]/4.0f*255.0f;
		bufferC[j] = (unsigned char)std::max(0.0f, std::min(255.0f, v));
	}
	writePPM(bufferC, name, w, h);
	delete[] bufferC;
}


void GLTools::writePPM(GLuint texId, char* name, int w, int h){
	float* buffer = new float[w*h];
	glBindTexture(GL_TEXTURE_2D, texId);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, buffer);
	glBindTexture(GL_TEXTURE_2D, 0);
	writePPM(buffer, name, w, h);
	delete[] buffer;
}

/*char* GLTools::readFile(const char* filename){

	// open file with read pointer at end of file
	ifstream file(filename, ios::in|ios::binary|ios::ate);

	if(!file.good()){
		stringstream s;
		s << "Couldn't open file " << filename << endl;
	}

	// read file into array
	int size = (int) file.tellg();
	char* memblock = new char [size+1];
	file.seekg (0, ios::beg);
	file.read (memblock, size);
	file.close();
	memblock[size] = '\0';

	// close file
	file.close();

	return memblock;
}*/

string GLTools::readFile(const char* filename){
	
	string includeString = "#pragma include ";

	string buffer;
	string line;
	ifstream file(filename);
	if(!file.good()){
		cout << "Could not open file " << filename << endl;
		return "";
	}
	while(getline(file,line)){
		//check if we need to read included files
		if (line.find(includeString, 0) != string::npos){
			string includeFilename = line.substr(includeString.size(), line.size());
			buffer += readFile(includeFilename.c_str());
		}

		buffer += line + "\n";
	}
	return buffer;
}


void GLTools::checkGLErrors(char const * file, int line, std::ostream& os) {
	GLuint errnum;
	char const * errstr;
	bool hasError = false;
	while (errnum = glGetError()){
		hasError = true;
		errstr = reinterpret_cast<const char *>(gluErrorString(errnum));
		if (errstr) {
			os << errstr; 
		} else {
			os << "Error " << errnum;
		}
		os << " at " << file << ":" << line << endl;
	}
}

void GLTools::printShaderInfoLog(GLuint shaderID){

  int infologLength = 0;
  int charsWritten  = 0;
  char *infoLog;
  
  glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH,&infologLength);
  
  if (infologLength > 1){
	infoLog = new char[infologLength];
	glGetShaderInfoLog(shaderID, infologLength, &charsWritten, infoLog);    
	stringstream s;
	s << "ShaderLog:\n" << infoLog << endl;    
	delete []infoLog;
  }
}

void GLTools::printProgramInfoLog(GLuint programID){

  int infologLength = 0;
  int charsWritten  = 0;
  char *infoLog;
  
  glGetProgramiv(programID, GL_INFO_LOG_LENGTH,&infologLength);
  
  if (infologLength > 1){
		infoLog = (char *)malloc(infologLength);
		glGetProgramInfoLog(programID, infologLength, &charsWritten, infoLog);
		stringstream s;
		s << "ShaderLog:\n" << infoLog << endl;
		cout << s.str();
		delete []infoLog;
	}
}

GLuint GLTools::createShaderProgram(const char* vsFilename, const char* fsFilename){

	// create program
	GLuint programID = glCreateProgram();

	// create and attach shaders
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	string source = readFile(vsFilename);
	const char* s = source.c_str();
	glShaderSource(vertexShader, 1, &s, NULL);
	glCompileShader(vertexShader);
	printShaderInfoLog(vertexShader);
	source.clear();
	glAttachShader(programID, vertexShader);    
	glDeleteShader(vertexShader);

	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	source = readFile(fsFilename);
	s = source.c_str();
	glShaderSource(fragmentShader, 1, &s, NULL);
	glCompileShader(fragmentShader);
	printShaderInfoLog(fragmentShader);
	source.clear();
	glAttachShader(programID, fragmentShader);    
	glDeleteShader(fragmentShader);

	// link program
	glLinkProgram(programID);

	// check program 
	printProgramInfoLog(programID);

	return programID;

}

GLuint GLTools::createShaderProgram(const char* vsFilename, const char* gsFilename, const char* fsFilename){

	// create program
	GLuint programID = glCreateProgram();

	// create and attach shaders
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	string source = readFile(vsFilename);
	const char* s = source.c_str();
	glShaderSource(vertexShader, 1, &s, NULL);
	glCompileShader(vertexShader);
	printShaderInfoLog(vertexShader);
	source.clear();
	glAttachShader(programID, vertexShader);    
	glDeleteShader(vertexShader);

	GLuint geometryShader = glCreateShader(GL_GEOMETRY_SHADER_EXT);
	source = readFile(gsFilename);
	s = source.c_str();
	glShaderSource(geometryShader, 1, &s, NULL);
	glCompileShader(geometryShader);
	printShaderInfoLog(geometryShader);
	source.clear();
	glAttachShader(programID, geometryShader);  
	glProgramParameteriEXT(programID, GL_GEOMETRY_VERTICES_OUT_EXT, 3); //maximum number of vertices the geometry shader will emit in one invocation
	glProgramParameteriEXT(programID, GL_GEOMETRY_INPUT_TYPE_EXT, GL_TRIANGLES);
	glProgramParameteriEXT(programID, GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_TRIANGLE_STRIP);
	glDeleteShader(geometryShader);

	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	source = readFile(fsFilename);
	s = source.c_str();
	glShaderSource(fragmentShader, 1, &s, NULL);
	glCompileShader(fragmentShader);
	printShaderInfoLog(fragmentShader);
	source.clear();
	glAttachShader(programID, fragmentShader);    
	glDeleteShader(fragmentShader);

	// link program
	glLinkProgram(programID);

	// check program 
	printProgramInfoLog(programID);

	return programID;
}

