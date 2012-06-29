/*
Dummy vertex shader that passes through vertex position as final position.
Used so that geometry shader has access to original vertex position and can
then perform the final transformation.
*/

void main() {
	gl_Position =  gl_Vertex;
	gl_TexCoord[0]  = gl_MultiTexCoord0;
}

