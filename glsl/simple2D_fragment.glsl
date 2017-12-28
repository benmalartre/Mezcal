#version 130

// texture coordinates from vertex shaders
in vec2 st;

// texture sampler
uniform sampler2D tex;

// output fragment colour RGBA
out vec4 frag_colour;

void main () {
	frag_colour = vec4(texture(tex,st).rgb,1.0);
}
