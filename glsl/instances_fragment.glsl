#version 330
uniform int selected;
in vec3 inColor;
in vec3 inUVWs;
in vec3 inNorm;

out vec4 outColor;

uniform sampler2D tex;
uniform vec3 datas;

void main(){
	/*
	vec4 t = texture(tex,inUVWs.xz);
	float a = 0.0;
	if((t.r+t.g+t.b)<0.5)a = 1.0;
	outColor = vec4(inColor*0.1+vec3(0.666,0.666,0.666),a);
	*/
	vec3 nn = normalize((inNorm+1)/2);
	vec3 uv = normalize((inUVWs+1)/2);
	//outColor = vec4(inUVWs,1.0);// * vec4(nn,1);
	outColor = vec4(inColor,1.0);//vec4(texture(tex,uv.xz).xyz,1.0) * vec4(inColor,1.0);
	//outColor = vec4(inColor,1.0);
	//outColor = vec4(texture(tex,inUVWs.xz).xyz,1.0);//vec4(inUVWs,1.0);//*d;//vec4(inColor,1.0)* vec4((inNorm+1)/2,1);
	//outColor = vec4(inColor,1.0);
}
