#version 330

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 uvw;
layout (location = 2) in vec3 normal;

const int NUM_CASCADES = 3;

uniform mat4 MVP;
uniform mat4 lightsMVP[NUM_CASCADES];
uniform mat4 model;

out vec4 lights_space_pos[NUM_CASCADES];
out vec3 vertex_normal;
out vec3 vertex_position;
out float clip_space_pos_z;

void main()
{
    vec4 pos = vec4(position, 1.0);

    gl_Position = MVP * pos;
    clip_space_pos_z = gl_Position.z;

    for (int i = 0 ; i < NUM_CASCADES ; i++) {
        lights_space_pos[i] = lightsMVP[i] * pos;
    }

    vertex_normal = (model * vec4(normal, 0.0)).xyz;
    vertex_position = (model * vec4(position, 1.0)).xyz;
}