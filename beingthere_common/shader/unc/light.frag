/*
Diffuse light with attenuation
*/

varying float nDotL;
varying float distanceToCam;
uniform sampler2D texColor;

#define GAIN 3.0

void main()
{

	float att = 1.0 / (gl_LightSource[0].constantAttenuation +
			   gl_LightSource[0].linearAttenuation * distanceToCam +
			   gl_LightSource[0].quadraticAttenuation * distanceToCam * distanceToCam);

	float value = nDotL * att * GAIN; //GAIN used to spread out values over 8-bit range 
	value = clamp(value, 1.0/255.0, 1.0); //clamp quality >0:1

	gl_FragColor =  vec4(texture2D(texColor,gl_TexCoord[0].st).rgb, value);
}

