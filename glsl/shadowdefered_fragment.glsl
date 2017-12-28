#version 330

out vec4 outColor;
in vec2 texCoords;

uniform sampler2D position_map;
uniform sampler2D normal_map;
uniform sampler2D color_map;
uniform sampler2D shadow_map;

uniform vec3 camera_position;
uniform mat4 light_view;
uniform mat4 light_proj;
uniform mat4 view_rotation_matrix;
uniform mat4 view;

const float bias2 = 0.001;

// This define the value to move one pixel left or right
uniform float x_pixel_offset;	
// This define the value to move one pixel up or down
uniform float y_pixel_offset ;

vec3 getShadowCoords(vec3 dir)
{
	mat4 inv_view = inverse(view);
    mat4 projected = light_proj * light_view * inv_view;
	vec4 proj_dir = projected * vec4(dir,1);
	
    proj_dir = proj_dir/proj_dir.w;
	return proj_dir.xyz;
}

float lookup( vec3 coords,vec2 offset)
{
	vec2 textureCoordinates = coords.xy * vec2(0.5,0.5) + vec2(0.5,0.5);

	offset *= vec2(x_pixel_offset,y_pixel_offset);
    const float bias = -0.005;
    float depth_value = texture( shadow_map, textureCoordinates + offset ).r ;
	if(depth_value>=1.0)return 1.0;
	if(coords.z * 0.5 + 0.5 < depth_value-bias)
	return 1.0;
	else return 0.0;
}

mat4 extractRotationMatrix(mat4 m){
return mat4(m[0].xyzw,
			m[1].xyzw,
			m[2].xyzw,
			0.0,0.0,0.0,1.0);
}

void main( void )
{
    // Read the data from the textures
    vec4 color = texture( color_map, texCoords );
	if(color.a == 0.0) discard;
	
    vec4 position = texture( position_map, texCoords );
    vec3 eye_dir = position.xyz - camera_position;

	vec3 shadow_coords = getShadowCoords(eye_dir);
	float s = 0.0;
	
	s =  lookup(shadow_coords,vec2(0.0,0.0));
	/*
	float x,y;
		for (y = -1.5 ; y <=1.5 ; y+=1.0)
			for (x = -1.5 ; x <=1.5 ; x+=1.0)
				s += lookup(shadow_coords,vec2(x,y));
		
	s /= 16.0 ;
	
	
	float x,y;
	for (y = -30.5 ; y <=30.5 ; y+=10.0)
		for (x = -30.5 ; x <=30.5 ; x+=10.0)
			s += lookup(shadow_coords,vec2(x,y));
	
	s /= 64.0 ;
	*/
	
	// use modulo to vary the sample pattern
	vec2 o = mod(floor(shadow_coords.xy), 2.0);

	s += lookup(shadow_coords,vec2(-1.5, 1.5) + o);
	s += lookup(shadow_coords,vec2( 0.5, 1.5) + o);
	s += lookup(shadow_coords,vec2(-1.5, -0.5) + o);
	s += lookup(shadow_coords,vec2( 0.5, -0.5) + o);
	s *= 0.25 ;
	
	s = max(s,0.25);
	
	outColor = vec4(s,s,s,1.0);
	//outColor = texture(shadow_map,texCoords);
	/*
    float diffuse_light = 0.25;
	if(max(dot(normal,vec4(light_dir,1.0)),0)>0.0) diffuse_light = 1.0;
	outColor = color *vec4(diffuse_light)*s ;
    */
	//outColor = vec4(s);
	/*
    float ambient_light = 0.1;

    outColor = (diffuse_light + ambient_light ) * image + pow(max(dot(light_dir,reflected_eye_vector),0.0), 100) * 1.5 * shadow;
	*/
}