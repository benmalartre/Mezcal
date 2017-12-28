#version 330

mat4 directionFromTwoVectors(vec3 dir,vec3 up){

	dir = normalize(dir);
	up = normalize(up);
	vec3 norm = cross(dir,up);
	norm = normalize(norm);
	
	up = cross(norm,dir);
	up = normalize(up);
	
	return mat4(up.x,dir.x,norm.x,0.0,
				up.y,dir.y,norm.y,0.0,
				up.z,dir.z,norm.z,0.0,
				0.0,0.0,0.0,1.0);
	/*
	return mat4(norm.x,up.x,-dir.x,0.0,
				norm.y,up.y,-dir.y,0.0,
				norm.z,up.z,-dir.z,0.0,
				0.0,0.0,0.0,1.0);
	*/
				
}
uniform int selected; 
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

layout(location=0) in vec3 s_pos;
layout(location=1) in vec3 s_norm;
layout(location=2) in vec3 s_uvws;
layout(location=3) in vec3 position;
layout(location=4) in vec3 normal;
layout(location=5) in vec3 tangent;
layout(location=6) in vec4 color;
layout(location=7) in vec3 scale;
layout(location=8) in float size;

void main(){

	vec3 tmp = vec3(0,0,1);
	mat4 rot = directionFromTwoVectors(normal,tmp);
	vec4 sshape = vec4(s_pos*scale*size,1.0)*rot;
	vec4 rshape = vec4(position+sshape.xyz,1.0);
	gl_Position = projection * view * model * rshape;
}
