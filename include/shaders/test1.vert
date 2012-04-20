in vec3 vertex;
varying vec3 normal;

void main() {
  gl_Position = gl_ModelViewProjectionMatrix * vec4(vertex, 1.0);
  
//  float Angle = radians(-90.0);
//  mat4 RotationMatrix = mat4(
//    cos( Angle ), sin( Angle ), 0.0, 0.0,
//    -sin( Angle ),  cos( Angle ), 0.0, 0.0,
//    0.0,           0.0,          1.0, 0.0,
//    0.0,           0.0,          0.0, 1.0 );

  vec3 temp = gl_NormalMatrix * gl_Normal;
  normal = vec3 (-temp.x, temp.y, -temp.z);
}

