#version 330

in vec4 shadow_coords;
uniform sampler2DShadow shadow_map;
//uniform sampler2D shadow_map;

out vec4 color;
const float bias2 = 0.005;

// This define the value to move one pixel left or right
uniform float x_pixel_offset;	
// This define the value to move one pixel up or down
uniform float y_pixel_offset ;

float lookup( vec2 offset)
{
	vec2 scale = textureSize(shadow_map,0);
	vec4 shadow_coords_d = shadow_coords*1/shadow_coords.w;
	shadow_coords_d += vec4(offset.x * x_pixel_offset, offset.y * y_pixel_offset,bias2,1.0);
	//if(texture(shadow_map,shadow_coords_d.xy).z<shadow_coords_d.z)return 1.0;
	//else return 0.0;
	return texture( shadow_map, shadow_coords_d.xyz);
}

void main(){
	vec4(1.0,0.0,0.0,1.0);
}
void main2()
{	
	// Used to lower moiré pattern and self-shadowing
	//shadowCoordinateWdivide.z += ;
	
	float shadow=0.2;
	
	// Avoid counter shadow
	//if (shadow_coords.w > 1.0)
	//{
		// Simple lookup, no PCF
					shadow = lookup(vec2(0.0,0.0));
					//shadow = CalcShadowFactor(shadow_coords);
		

		// 8x8 kernel PCF
					/*
					float x,y;
					for (y = -3.5 ; y <=3.5 ; y+=1.0)
						for (x = -3.5 ; x <=3.5 ; x+=1.0)
							shadow += lookup(vec2(x,y));
					
					shadow /= 64.0 ;
					*/


		// 8x8 PCF wide kernel (step is 10 instead of 1)
				/*
					float x,y;
					for (y = -30.5 ; y <=30.5 ; y+=10.0)
						for (x = -30.5 ; x <=30.5 ; x+=10.0)
							shadow += lookup(vec2(x,y));
					
					shadow /= 64.0 ;
				*/
	



		// 4x4 kernel PCF
		/*
		float x,y;
		for (y = -1.5 ; y <=1.5 ; y+=1.0)
			for (x = -1.5 ; x <=1.5 ; x+=1.0)
				shadow += lookup(vec2(x,y));
		
		shadow /= 16.0 ;
		*/
		


		// 4x4  PCF wide kernel (step is 10 instead of 1)
					/*
					float x,y;
					for (y = -10.5 ; y <=10.5 ; y+=10.0)
						for (x = -10.5 ; x <=10.5 ; x+=10.0)
							shadow += lookup(vec2(x,y));
					
					shadow /= 16.0 ;
					*/
		
		// 4x4  PCF dithered
					/*
					// use modulo to vary the sample pattern
					vec2 o = mod(floor(gl_FragCoord.xy), 2.0);
				
					shadow += lookup(vec2(-1.5, 1.5) + o);
					shadow += lookup(vec2( 0.5, 1.5) + o);
					shadow += lookup(vec2(-1.5, -0.5) + o);
					shadow += lookup(vec2( 0.5, -0.5) + o);
					shadow *= 0.25 ;
					*/
	//}
  	color =	  vec4(shadow+0.2,shadow+0.2,shadow+0.2,1.0);
  
}

/*
void main () {
	vec4 shadow_coords_d = shadow_coords / shadow_coords.w;

	float visibility = 1.0;
	if ( texture( shadow_map, shadow_coords_d.xy ).z  <  shadow_coords_d.z-bias2){
		visibility = 0.2;
	}

	color = vec4(visibility,visibility,visibility,1.0);
}
*/