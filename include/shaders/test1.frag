#version 140
varying vec3 normal;
uniform int width;
uniform int height;
uniform samplerBuffer kinectTexture;

void main(void)
{
  //gl_FragData[0] = length (normal);
  //normal = normalize (normal);
  vec2 uv = vec2 (gl_FragCoord.x/width, gl_FragCoord.y/height);
  float depth = texelFetch (kinectTexture, int(gl_FragCoord.y)*width + int(gl_FragCoord.x)).x;
  gl_FragData[0].x = depth;
  gl_FragData[0].y = depth;
  gl_FragData[0].z = depth;

  //gl_FragData[1] = 2.0 - length (normal);
  //gl_FragData[1] = vec4(1.0, 1.0, 0.0, 1.0);
  gl_FragData[2] = vec4(
    (normal.x + 1.0) * 0.5,
    (normal.y + 1.0) * 0.5,
    (normal.z + 1.0) * 0.5,
    1.0);
}

		
