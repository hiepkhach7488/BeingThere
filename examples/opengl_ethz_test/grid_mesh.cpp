#include "grid_mesh.h"
#include <GL/glew.h>
#include <vector>


using namespace std;


GridMesh::GridMesh(int width, int height):_width(width), _height(height){
	createBufferObjects();
}

GridMesh::~GridMesh(){
	glDeleteBuffers(1, &_vbo);
	glDeleteBuffers(1, &_ibo);
}

void GridMesh::resize(int width, int height){

	//delete old
	glDeleteBuffers(1, &_vbo);
	glDeleteBuffers(1, &_ibo);

	//create new
	_width = width;
	_height = height;
	createBufferObjects();
}

void GridMesh::createBufferObjects(){

	//create vertex buffer object
	vector<float> vertices;
	vector<unsigned int> indices;
	for(int i=0; i<_height; i++){
		for(int j=0; j<_width; j++){
			vertices.push_back((float) j);
			vertices.push_back((float) i);
			vertices.push_back(0.0f);
		}
	}
	glGenBuffers(1, &_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, _vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertices.size(), &(vertices.at(0)), GL_STATIC_DRAW); 
	vertices.clear();

	//create index buffer object
	for(int i=0; i < _width*(_height-1); i++){
		if((i+1) % _width == 0){
			continue;
		}
		indices.push_back(i);
		indices.push_back(i+_width);
		indices.push_back(i+1);

		indices.push_back(i+1);
		indices.push_back(i+_width);
		indices.push_back(i+_width+1);
	}
	glGenBuffers(1, &_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*indices.size(), &(indices.at(0)), GL_STATIC_DRAW);
	_nrElements = (int) indices.size();
	indices.clear();
}
