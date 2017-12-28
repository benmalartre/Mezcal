#version 330

in vec3 vertex_position;
in vec3 vertex_normal;
in vec3 vertex_color;

uniform float nearplane;
uniform float farplane;

uniform sampler2D tex;
uniform mat4 view;

layout(location=0) out vec4 position;
layout(location=1) out vec4 normal;
layout(location=2) out vec4 color;

float linearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // Back to NDC 
    return (2.0 * nearplane * farplane) / (farplane + nearplane - z * (farplane - nearplane));	

}

void main()
{    
    // Store the fragment position vector in the first gbuffer texture
    position.xyz = vertex_position;
	
    // And store linear depth into gPositionDepth's alpha component
    position.a = linearizeDepth(gl_FragCoord.z); // Divide by FAR if you need to store depth in range 0.0 - 1.0 (if not using floating point colorbuffer)
	
	// Also store the per-fragment normals into the gbuffer
    normal = vec4(normalize(vertex_normal),0.0);
	
    // And the diffuse per-fragment color
    color = vec4(vertex_color,1.0); // Currently all objects have constant albedo color
}
