#ifndef GRID_MESH_H_
#define GRID_MESH_H_


//a triangle mesh as a grid

class GridMesh
{
public:
	GridMesh(int width, int height);
	virtual ~GridMesh();

	unsigned int getVbo(){ return _vbo; };
	unsigned int getIbo(){ return _ibo; };
	int getNrElements() {return _nrElements; };

	void resize(int width, int height);

	int getWidth() { return _width;};
	int getHeight() { return _height;};

protected:

private:

	unsigned int _vbo; //vertex buffer object
	unsigned int _ibo; //index buffer object
	int _nrElements;
	int _width;
	int _height;

	void createBufferObjects();


};

#endif /*GRID_MESH_H_*/
