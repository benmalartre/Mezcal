#version 330 core
out vec4 outColor;
in vec2 texCoords;

uniform sampler2D position_map;
uniform sampler2D normal_map;
uniform sampler2D noise_map;
#define MAX_KERNEL_SIZE 64
uniform vec3 kernel_samples[MAX_KERNEL_SIZE];
uniform vec2 noise_scale;
uniform int kernel_size;
uniform float occ_radius;
uniform int occ_power;

uniform mat4 view;
uniform mat4 projection;

void main()
{

    // Get input for SSAO algorithm
    vec3 fragPos = texture(position_map, texCoords).xyz;
    float depth = texture(position_map,texCoords).w;
    if(depth>0.99999)
    {
        vec3 normal = texture(normal_map, texCoords).rgb;
        vec3 randomVec = texture(noise_map, texCoords * noise_scale).xyz;
	
        // Create TBN change-of-basis matrix: from tangent-space to view-space
        vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
        vec3 bitangent = cross(normal, tangent);

        mat3 TBN = mat3(tangent, bitangent, normal);
	
        // Iterate over the sample kernel and calculate occlusion factor
        float occlusion = 0.0;
        for(int i = 0; i < kernel_size; ++i)
        {
            // get sample position
            vec3 sample = TBN * kernel_samples[i]; // From tangent to view-space
            sample = fragPos + sample * occ_radius;
        
            // project sample position (to sample texture) (to get position on screen/texture)
            vec4 offset = vec4(sample, 1.0);
            offset = projection * offset; // from view to clip-space
            offset.xyz /= offset.w; // perspective divide
            offset.xyz = offset.xyz * 0.5 + 0.5; // transform to range 0.0 - 1.0
        
            // get sample depth
            float sampleDepth = -texture(position_map, offset.xy).w; // Get depth value of kernel sample
        
            // range check & accumulate
            float rangeCheck = smoothstep(0.0, 1.0, occ_radius / abs(fragPos.z - sampleDepth ));
            occlusion += (sampleDepth >= sample.z ? 1.0 : 0.0) * rangeCheck;
	
        }
        occlusion = pow(1.0 - (occlusion / kernel_size),occ_power+1);
    
        outColor = vec4(occlusion);
    }
    else
        outColor = vec4(1.0,0.0,0.0,1.0);

}