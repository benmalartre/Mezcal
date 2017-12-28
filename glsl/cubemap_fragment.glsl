#version 330
in vec3 texcoords;
uniform samplerCube cube_tex;
out vec4 outColor;
void main(){
	outColor = texture(cube_tex,texcoords);
}