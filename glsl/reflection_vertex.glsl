#version 400

in vec3 position; // positions from mesh
in vec3 normal; // normals from mesh
uniform mat4 projection, view, model; // proj, view, model matrices
out vec3 pos_eye;
out vec3 n_eye;

void main () {
  pos_eye = vec3 (view * model * vec4 (position, 1.0));
  n_eye = vec3 (view * model * vec4 (normal, 0.0));
  gl_Position = projection * view * model * vec4 (position, 1.0);
}