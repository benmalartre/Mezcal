#version 330

uniform mat4 view;
uniform mat4 projection;
uniform mat4 model;

uniform float nearplane;
uniform float farplane;


layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec3 tangent;
layout(location=3) in vec3 uvws;
layout(location=4) in vec4 color;

out vec3 vertex_position;
out vec3 vertex_normal;
out vec3 vertex_tangent;
out vec3 vertex_uvws;
out vec4 vertex_color;

void main(){
	vec4 viewPos =  view*model * vec4(position, 1.0f);
    vertex_position = viewPos.xyz; 
    gl_Position = projection * viewPos;
    vertex_color = color;
	
    //mat3 normalMatrix = transpose(inverse(mat3(view* model)));
    //vertex_normal = normalMatrix * normal;
    vertex_normal = normal;
	vec4 p = model * vec4(position,1.0f);
}
 
