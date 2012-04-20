varying vec3 normal;

void main(void)
{
  gl_FragData[0] = length (normal);
  normal = normalize (normal);
  gl_FragData[1] = 2.0 - length (normal);
  //gl_FragData[1] = vec4(1.0, 1.0, 0.0, 1.0);
  gl_FragData[2] = vec4(
    (normal.x + 1.0) * 0.5,
    (normal.y + 1.0) * 0.5,
    (normal.z + 1.0) * 0.5,
    1.0);
}

		
