#version 130
out vec4 normal;

void main() {
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

  vec3 temp = gl_NormalMatrix * gl_Normal;
  normal = vec4 (-temp.x, temp.y, -temp.z, 1.0);
}

