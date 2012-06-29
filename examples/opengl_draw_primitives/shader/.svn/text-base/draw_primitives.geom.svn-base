#version 410

precision highp float;

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in vec4 color[];
out vec4 geom_color;

void main(void){	
   for (int n = 0; n < 3; n++) {
        gl_Position = gl_in[n].gl_Position;
        geom_color = color[n];
        EmitVertex();
    }
    EndPrimitive();
}

