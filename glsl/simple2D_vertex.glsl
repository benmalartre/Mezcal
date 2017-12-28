#version 130

// vertex positions input attribute
in vec2 vp;

// per-vertex texture coordinates input attribute
in vec2 vt;


uniform float tanhalffov;
uniform float aspectratio;

// texture coordinates to be interpolated to fragment shaders
out vec2 st;
out vec3 view_ray;

void main () {
	// interpolate texture coordinates
	st = vt;
	// transform vertex position to clip space (camera view and perspective)
	gl_Position = vec4 (vp,0.0, 1.0);
	view_ray = vec3(st.x * tanhalffov * aspectratio, st.y * tanhalffov,1.0);
}
