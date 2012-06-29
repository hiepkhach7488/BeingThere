#version 330 compatibility

layout(location = 0) in vec4 InPosition;
layout(location = 1) in vec4 InTexCoords;
layout(location = 2) in float InNDotL;
layout(location = 3) in float InDistanceToCam;

out float nDotL;
out float distanceToCam;

void main()
{	
	gl_Position = InPosition;
	gl_TexCoord[0] = InTexCoords;
	nDotL = InNDotL;
	distanceToCam = InDistanceToCam;
}
