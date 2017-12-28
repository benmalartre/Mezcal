#version 330

in vec3 vertex_position;
in vec3 vertex_normal;
in vec3 vertex_tangent;
in vec3 vertex_uvws;
in vec4 vertex_color;
    

uniform sampler2D tex;

//We need view matrix
uniform mat4 view;
uniform vec3 lightPosition;
uniform int selected;
uniform int wireframe;
uniform vec3 wirecolor;

out vec4 outColor;

void main()
{

	if(wireframe == 1){
		if(selected == 1)outColor = vec4(1.0,1.0,1.0,1.0);
		else outColor = vec4(wirecolor,1.0);
	}
	else{

		//Raise Light Position to eye space
		vec3 light_position_eye = vec3(view*vec4(lightPosition,1.0));
		vec3 distance_to_light_eye = light_position_eye - vertex_position;

		vec3 direction_to_light_eye = normalize(distance_to_light_eye);
		vec3 direction_to_camera = normalize( view[3].xyz);
		float d;
		//if(dot(direction_to_camera,normalize(vertex_normal))<0){
			d = 1.0*dot(direction_to_light_eye,normalize(vertex_normal))+0.5;
			d = max(d,0.2);
		//}
		//else
		//	d = 0.2;
	
		float a = 1.0;
		
		vec4 t = texture(tex,vertex_uvws.xz);//*d;
		if((t.x+t.y+t.z)/3>0.5)a=0.0;
		vec3 color = vec3(d);//vertex_color.xyz*d;
		outColor = vec4(color,a);

	}

}