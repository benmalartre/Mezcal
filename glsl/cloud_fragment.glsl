#version 330
uniform int selected;
in vec4 inColor;
in vec3 inUVWs;
in vec3 inNorm;

out vec4 outColor;

uniform sampler2D tex;
uniform vec3 datas;

void main(){
	outColor = inColor;// * vec4(nn,1);
}
