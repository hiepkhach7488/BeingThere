#version 150

precision highp float;

layout (points) in;
layout (points, max_vertices = 1) out;

in vec4 v_color[];
in vec2 tex_coord[];
out vec4 color;

void main(void){
	float dx = tex_coord[0].x - 320.0;
	float dy = tex_coord[0].y - 240.0;
	float  d = sqrt(dx*dx + dy*dy);

	if(d < 480){
        for (int n = 0; n < 1; n++) {
            gl_Position = gl_in[n].gl_Position;
            color = v_color[n];
            EmitVertex();
        }

    EndPrimitive();
    }

}
