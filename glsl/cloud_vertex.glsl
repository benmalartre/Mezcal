#version 330

mat4 rotationMatrix(vec3 axis, float angle){
	axis = normalize(axis);
	float s = sin(angle);
	float c = cos(angle);
	float oc = 1.0 - c;
	return mat4(oc*axis.x*axis.x+c,			oc*axis.x*axis.y-axis.z*s,		oc*axis.z*axis.x+axis.y*s, 	0.0,
				oc*axis.x*axis.y+axis.z*s,	oc*axis.y*axis.y+c,				oc*axis.y*axis.z-axis.x*s,	0.0,
				oc*axis.z*axis.x-axis.y*s,	oc*axis.y*axis.z+axis.x*s,		oc*axis.z*axis.z+c,			0.0,
				0.0,						0.0,							0.0,						1.0);
}

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

layout(location=0) in vec3 position;
layout(location=1) in vec3 velocity;
layout(location=2) in vec3 normal;
layout(location=3) in vec3 tangent;
layout(location=4) in vec3 scale;
layout(location=5) in vec4 color;
layout(location=6) in float size;

out vec4 inColor;
out vec3 inVelocity;
out vec3 inUpdate;
void main(){
	
	if(selected ==1)
		//inColor = vec3(gl_InstanceID%10*0.1+0.2,0.0,0.0);
		inColor = color;
	else 	
		inColor = color;

	vec3 tmp = vec3(0,0,1);
	gl_Position = projection * view * model * vec4(position,1.0);
	inVelocity = velocity;
	inUpdate = scale * size;
}
