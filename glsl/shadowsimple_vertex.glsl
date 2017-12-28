#version 330

uniform mat4 bias;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 light_proj;
uniform mat4 light_view;
uniform mat4 model;

layout (location=0) in vec3 position;

out vec4 shadow_coords;

void main(){
	vec4 p = model * vec4(position,1.0);
	shadow_coords = bias * light_proj * light_view * p;
	gl_Position =  projection * view *  p;
	
}
 
