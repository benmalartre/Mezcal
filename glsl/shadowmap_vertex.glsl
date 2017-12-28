#version 330

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

layout (location = 0) in vec3 position;
//out float depth;

void main(){
	gl_Position = projection * view * model * vec4(position,1.0);
}
