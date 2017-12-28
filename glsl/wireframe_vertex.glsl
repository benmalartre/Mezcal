#version 330
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 offset;
in vec3 position;
void main(){
	vec4 v = view * model * vec4(position, 1.0);
	v.xyz = v.xyz * 0.999;

	gl_Position = projection * v;
	//gl_Position = projection * view *  model *offset*vec4(position,1.0);
}
