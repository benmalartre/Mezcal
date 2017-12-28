#version 330

const int NUM_CASCADES = 3;

in vec4 lights_space_pos[NUM_CASCADES];
in float clip_space_pos_z;

uniform sampler2D shadow_maps[NUM_CASCADES];
uniform float cascade_ends[NUM_CASCADES];

out vec4 outColor;


float calcShadowFactor(int cascade_index, vec4 light_space_pos)
{ 
    vec3 proj_coords = light_space_pos.xyz / light_space_pos.w; 

    vec2 uv_coords; 
    uv_coords.x = 0.5 * proj_coords.x + 0.5; 
    uv_coords.y = 0.5 * proj_coords.y + 0.5; 

    float z = 0.5 * proj_coords.z + 0.5; 
    float depth = texture(shadow_maps[cascade_index], uv_coords).x; 

    if (depth < z + 0.00001) 
        return 0.5;
    else 
        return 1.0; 
} 

void main()
{ 
    float shadow_factor = 0.0;

    for (int i = 0 ; i < NUM_CASCADES ; i++) {
        if (clip_space_pos_z <= cascade_ends[i]) {
            shadow_factor = calcShadowFactor(i, lights_space_pos[i]);
            break;
        }
    }
    outColor = vec4(shadow_factor, shadow_factor, shadow_factor, 1.0);
}