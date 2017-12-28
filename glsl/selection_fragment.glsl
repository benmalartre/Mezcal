#version 330
uniform vec3 uniqueID;
out vec4 outColor;
void main()
{
	outColor = vec4(uniqueID,1.0);
}
