#version 130 
#extension GL_EXT_gpu_shader4 : enable

precision highp float;

#define GAIN 3.0

varying float nDotL;
varying float distanceToCam;

uniform sampler2D texColor;

in vec2 TexCoord;
out vec4 output_color;

void main() {
	float att = 1.0 / (gl_LightSource[0].constantAttenuation +
			   gl_LightSource[0].linearAttenuation * distanceToCam +
			   gl_LightSource[0].quadraticAttenuation * distanceToCam * distanceToCam);

	float value = nDotL * att * GAIN; //GAIN used to spread out values over 8-bit range 
	value = clamp(value, 1.0/255.0, 1.0); //clamp quality >0:1

	output_color = vec4(texture2D(texColor, TexCoord.st).rgb, value);
	//output_color = texture2D(texColor, TexCoord.st);
}

