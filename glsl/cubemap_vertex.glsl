#version 330
in vec3 coords;
uniform mat4 P, V;
out vec3 texcoords;
void main(void){
	texcoords = coords;
	gl_Position = P * V * vec4(coords.xyz,1.0);
}